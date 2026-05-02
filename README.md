# SmartBall: Sensor-Equipped Ball for Sports Analytics

This repository contains a reconstructed implementation of the SmartBall final project for CIS4930 Wireless & Mobile Computing. The original source code was not preserved, so this version was rebuilt from the final course report, presentation, and demo materials to match the documented system architecture and behavior as closely as possible. [file:13][file:1][file:12]

## Overview

SmartBall is a low-cost sports analytics prototype that embeds sensing and wireless communication directly inside a game ball. The system uses an ESP32 microcontroller and an MPU6050 IMU to detect throws, collect motion data, and transmit that data over Wi-Fi using UDP to a base station running a Python listener. [file:13][file:1]

The listener processes incoming throw samples in real time and computes:
- Release time
- Release speed
- Peak acceleration
- Peak rotational velocity
- Expected throw distance
- Throw strength classification [file:13][file:1]

## System Architecture

The project has two main parts:

1. **ESP32 sender**
   - Reads MPU6050 accelerometer and gyroscope data
   - Calibrates gyro bias at startup
   - Maintains a circular pre-roll buffer
   - Detects a throw using acceleration or angular velocity thresholds
   - Sends throw data over UDP using `START` and `END` markers [file:13]

2. **Python listener**
   - Listens on UDP port `4210`
   - Collects timestamped IMU samples during each throw
   - Detects the release point from acceleration behavior
   - Estimates release speed by integrating linear acceleration
   - Predicts expected throw distance using a calibrated quadratic model
   - Logs each processed throw to CSV [file:13][file:12]

## Files

- `smartball_esp32.ino` — ESP32 Arduino code for throw detection and UDP streaming [file:13]
- `smartball_listener.py` — Python listener for real-time throw processing and CSV logging [file:13]
- `Course-Project-Report.pdf` — final written report describing the full system and algorithm [file:13]
- `Wireless-Final-Project.pptx` — presentation slides summarizing the design and results [file:1]

## Hardware

The documented prototype uses:
- ESP-32S microcontroller
- MPU6050 accelerometer/gyroscope
- MT3608 DC-DC boost converter
- 3.7V 3000mAh LiPo battery
- Toggle switches
- TP4056 USB-C charger module [file:13]

The report states that the MPU6050 is connected to the ESP32 using:
- `SDA_PIN = 25`
- `SCL_PIN = 27` [file:13]

## Documented ESP32 Behavior

According to the report, the ESP32 side uses:
- Sampling rate: 200 Hz via `SAMPLE_US = 5000`
- Pre-roll buffer: 50 samples
- Acceleration trigger: `18.0 m/s^2`
- Gyroscope trigger: `6.0 rad/s`
- Post-trigger transmission window: 900 ms
- Cooldown period: 1500 ms
- Gyroscope calibration: 500 stationary samples [file:13]

Each UDP sample is sent in this format:

```text
t_ms, ax, ay, az, gx, gy, gz
```

The sender also transmits:
- `START` before throw samples
- `END` after throw capture finishes [file:13]

## Documented Listener Behavior

The listener described in the report uses:
- `MIN_SAMPLES = 25`
- `PRE_ROLL_SEC = 0.15`
- Minimum 8 pre-roll samples
- `WARN_PRE_GYRO_MEAN = 2.5`
- `FREE_FLIGHT_THRESH = 0.65 * 9.81`
- `FREE_FLIGHT_MIN_SEC = 0.020`
- `MAX_RELEASE_FRACTION = 0.90`
- `MIN_RELEASE_TIME_SEC = 0.12`
- `MAX_RELEASE_TIME_SEC = 0.95`
- `MAX_INTEGRATION_WINDOW_SEC = 0.30`
- Valid speed range: `0.5` to `10.0 m/s`
- Distance model: `expected_distance = 0.055 * speed^2 + 0.0` [file:13]

Throw strength is classified as:
- `weak` if speed < 2.0 m/s
- `medium` if speed is between 2.0 and 4.5 m/s
- `strong` if speed > 4.5 m/s [file:13]

## Results

The final report states that 38 valid throws were collected in a controlled environment at about 11 ft (3.35 m), and the system achieved an average estimated throw distance of 3.48 m versus a true distance of 3.35 m, corresponding to roughly 4% average error. The average release speed was about 7.94 m/s, with average peak acceleration of 183.94 m/s² and average peak rotation of 26.14 rad/s. [file:13]

## Setup

### ESP32

1. Open `smartball_esp32.ino` in Arduino IDE.
2. Install required libraries:
   - `Adafruit_MPU6050`
   - `Adafruit Unified Sensor`
3. Update:
   - `WIFI_SSID`
   - `WIFI_PASS`
   - `LISTENER_IP`
4. Flash the sketch to the ESP32. [file:13]

### Python Listener

1. Install Python 3.
2. Install NumPy:
   ```bash
   pip install numpy
   ```
3. Run:
   ```bash
   python smartball_listener.py
   ```
4. The listener will wait for UDP packets on port `4210`. [file:13]

## Notes

This repository is a reconstructed implementation, not the original final codebase. It was rebuilt from the documented algorithms, constants, screenshots, presentation, and demo transcript after the original source code was unavailable. Some implementation details such as Wi-Fi credentials and any undocumented tuning choices may differ from the original project version. [file:13][file:1][file:12]

## Course

CIS4930-005 Wireless & Mobile Computing [file:13]

## Authors

Anthony Lozbin  
Pritkumar Pagda  
Jacob Moran [file:13]
