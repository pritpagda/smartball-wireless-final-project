#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
WiFiUDP udp;

// ---------- Wi-Fi / UDP ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
IPAddress LISTENER_IP(192, 168, 1, 100);
const uint16_t LISTENER_PORT = 4210;

// ---------- Hardware pins ----------
const int SDA_PIN = 25;
const int SCL_PIN = 27;

// ---------- Sampling / throw config ----------
const uint32_t SAMPLE_US = 5000;          // 200 Hz
const int PREBUFFER_SIZE = 50;            // 250 ms at 200 Hz
const float ACC_TRIGGER = 18.0f;          // m/s^2
const float GYRO_TRIGGER = 6.0f;          // rad/s
const uint32_t POST_RECORD_MS = 900;      // send live samples for 900 ms after trigger
const uint32_t COOLDOWN_MS = 1500;        // ignore new throws during cooldown
const int GYRO_CAL_SAMPLES = 500;

struct Sample {
  uint32_t t_ms;
  float ax, ay, az;
  float gx, gy, gz;
};

Sample prebuf[PREBUFFER_SIZE];
int prebufIndex = 0;
bool prebufFilled = false;

float gx_bias = 0.0f;
float gy_bias = 0.0f;
float gz_bias = 0.0f;

bool recording = false;
uint32_t lastSampleUs = 0;
uint32_t recordStartMs = 0;
uint32_t lastThrowEndMs = 0;

static inline float mag3(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  Serial.print("Connected. ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void sendPacket(const String& msg) {
  udp.beginPacket(LISTENER_IP, LISTENER_PORT);
  udp.print(msg);
  udp.endPacket();
}

void sendSample(const Sample& s) {
  char line[192];
  snprintf(
    line,
    sizeof(line),
    "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
    (unsigned long)s.t_ms,
    s.ax,
    s.ay,
    s.az,
    s.gx,
    s.gy,
    s.gz
  );
  sendPacket(String(line));
}

void pushPrebuffer(const Sample& s) {
  prebuf[prebufIndex] = s;
  prebufIndex = (prebufIndex + 1) % PREBUFFER_SIZE;
  if (prebufIndex == 0) prebufFilled = true;
}

void flushPrebuffer() {
  int count = prebufFilled ? PREBUFFER_SIZE : prebufIndex;
  int start = prebufFilled ? prebufIndex : 0;

  for (int i = 0; i < count; i++) {
    int idx = (start + i) % PREBUFFER_SIZE;
    sendSample(prebuf[idx]);
    delay(1);
  }
}

void calibrateGyro() {
  float sx = 0.0f, sy = 0.0f, sz = 0.0f;

  Serial.println("Keep ball still: calibrating gyro...");
  delay(1000);

  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    sx += gyro.gyro.x;
    sy += gyro.gyro.y;
    sz += gyro.gyro.z;
    delay(4);
  }

  gx_bias = sx / GYRO_CAL_SAMPLES;
  gy_bias = sy / GYRO_CAL_SAMPLES;
  gz_bias = sz / GYRO_CAL_SAMPLES;

  Serial.println("Gyro calibration done.");
  Serial.printf("Biases: %.6f %.6f %.6f\n", gx_bias, gy_bias, gz_bias);
}

Sample readSample() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Sample s;
  s.t_ms = millis();
  s.ax = accel.acceleration.x;
  s.ay = accel.acceleration.y;
  s.az = accel.acceleration.z;
  s.gx = gyro.gyro.x - gx_bias;
  s.gy = gyro.gyro.y - gy_bias;
  s.gz = gyro.gyro.z - gz_bias;
  return s;
}

void startRecording(const Sample& triggerSample) {
  recording = true;
  recordStartMs = millis();

  sendPacket("START");
  flushPrebuffer();
  sendSample(triggerSample);

  Serial.println("Throw detected -> streaming pre-roll + live data");
}

void stopRecording() {
  sendPacket("END");
  recording = false;
  lastThrowEndMs = millis();
  Serial.println("Throw capture complete");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  connectWiFi();
  udp.begin(LISTENER_PORT);
  calibrateGyro();
}

void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastSampleUs) < SAMPLE_US) {
    return;
  }
  lastSampleUs = nowUs;

  Sample s = readSample();
  float accMag = mag3(s.ax, s.ay, s.az);
  float gyroMag = mag3(s.gx, s.gy, s.gz);

  bool trigger = (accMag > ACC_TRIGGER) || (gyroMag > GYRO_TRIGGER);
  bool cooldownDone = (millis() - lastThrowEndMs) > COOLDOWN_MS;

  if (!recording && trigger && cooldownDone) {
    startRecording(s);
  } else if (recording) {
    sendSample(s);
    if ((millis() - recordStartMs) >= POST_RECORD_MS) {
      stopRecording();
    }
  }

  pushPrebuffer(s);
}
