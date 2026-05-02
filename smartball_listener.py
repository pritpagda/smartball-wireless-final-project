import csv
import os
import socket
from datetime import datetime

import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 4210
CSV_FILE = "smartball_throw_log.csv"

G = 9.81
MIN_SAMPLES = 25
PRE_ROLL_SEC = 0.15
MIN_PRE_SAMPLES = 8
WARN_PRE_GYRO_MEAN = 2.5
FREE_FLIGHT_THRESH = 0.65 * G
FREE_FLIGHT_MIN_SEC = 0.020
MAX_RELEASE_FRACTION = 0.90
MIN_RELEASE_TIME_SEC = 0.12
MAX_RELEASE_TIME_SEC = 0.95
MAX_INTEGRATION_WINDOW_SEC = 0.30
MIN_VALID_SPEED = 0.5
MAX_VALID_SPEED = 10.0
DIST_COEFF = 0.055
DIST_BIAS = 0.0


def moving_average(x, n=5):
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return x
    if len(x) < n:
        kernel = np.ones(len(x), dtype=float) / len(x)
        return np.convolve(x, kernel, mode="same")
    kernel = np.ones(n, dtype=float) / n
    return np.convolve(x, kernel, mode="same")


def classify_throw(speed):
    if speed < 2.0:
        return "weak"
    if speed <= 4.5:
        return "medium"
    return "strong"


def detect_release(acc_mag, dt, start_idx):
    acc_s = moving_average(acc_mag, 5)
    sustain_n = max(3, int(FREE_FLIGHT_MIN_SEC / dt))

    for i in range(start_idx, len(acc_s) - sustain_n):
        if np.all(acc_s[i:i + sustain_n] < FREE_FLIGHT_THRESH):
            return i

    d = np.diff(acc_s)
    if len(d) == 0:
        return 0
    return int(np.argmin(d))


def init_csv():
    if os.path.exists(CSV_FILE):
        return
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "logged_at",
            "sample_count",
            "median_dt_s",
            "release_idx",
            "release_time_s",
            "release_speed_mps",
            "peak_accel_mps2",
            "peak_rotation_rads",
            "expected_distance_m",
            "strength_label",
            "status",
        ])


def parse_sample_line(line):
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 7:
        return None
    try:
        vals = list(map(float, parts))
    except ValueError:
        return None
    return {
        "t_ms": vals[0],
        "ax": vals[1],
        "ay": vals[2],
        "az": vals[3],
        "gx": vals[4],
        "gy": vals[5],
        "gz": vals[6],
    }


def write_result(result):
    init_csv()
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            datetime.now().isoformat(timespec="seconds"),
            result["sample_count"],
            f"{result['median_dt']:.6f}",
            result["release_idx"],
            f"{result['release_time']:.6f}",
            f"{result['release_speed']:.6f}",
            f"{result['peak_acc']:.6f}",
            f"{result['peak_gyro']:.6f}",
            f"{result['expected_distance']:.6f}",
            result["strength"],
            result["status"],
        ])


def process_throw(samples):
    result = {
        "sample_count": len(samples),
        "median_dt": 0.0,
        "release_idx": -1,
        "release_time": 0.0,
        "release_speed": 0.0,
        "peak_acc": 0.0,
        "peak_gyro": 0.0,
        "expected_distance": 0.0,
        "strength": "n/a",
        "status": "ok",
    }

    if len(samples) < MIN_SAMPLES:
        result["status"] = "insufficient_sample_count"
        return result

    arr = np.array([
        [s["t_ms"], s["ax"], s["ay"], s["az"], s["gx"], s["gy"], s["gz"]]
        for s in samples
    ], dtype=float)

    t_ms = arr[:, 0]
    t = (t_ms - t_ms[0]) / 1000.0

    if len(t) < 2:
        result["status"] = "insufficient_sample_count"
        return result

    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        dt = 0.005
    result["median_dt"] = dt

    acc = arr[:, 1:4]
    gyr = arr[:, 4:7]

    acc_mag = np.linalg.norm(acc, axis=1)
    gyr_mag = np.linalg.norm(gyr, axis=1)

    peak_acc = float(np.max(acc_mag))
    peak_gyro = float(np.max(gyr_mag))
    result["peak_acc"] = peak_acc
    result["peak_gyro"] = peak_gyro

    pre_n = max(MIN_PRE_SAMPLES, int(PRE_ROLL_SEC / dt))
    pre_n = min(pre_n, len(samples) - 1)
    if pre_n <= 0:
        result["status"] = "insufficient_sample_count"
        return result

    g_body = np.mean(acc[:pre_n], axis=0)
    lin_acc = acc - g_body

    pre_gyro_mean = float(np.mean(gyr_mag[:pre_n]))
    if pre_gyro_mean > WARN_PRE_GYRO_MEAN:
        result["status"] = "ok_warn_preroll"

    release_idx = detect_release(acc_mag, dt, pre_n)
    result["release_idx"] = int(release_idx)

    if release_idx <= pre_n or release_idx >= len(samples) - 2:
        result["status"] = "bad_release_index"
        return result

    if release_idx >= int(MAX_RELEASE_FRACTION * len(samples)):
        result["status"] = "release_too_late"
        return result

    release_time = float(t[release_idx])
    result["release_time"] = release_time

    if release_time < MIN_RELEASE_TIME_SEC or release_time > MAX_RELEASE_TIME_SEC:
        result["status"] = "release_time_out_of_range"
        return result

    max_window_n = max(1, int(MAX_INTEGRATION_WINDOW_SEC / dt))
    start_idx = max(pre_n, release_idx - max_window_n)

    v = np.array([0.0, 0.0, 0.0], dtype=float)
    for i in range(start_idx + 1, release_idx + 1):
        dti = t[i] - t[i - 1]
        v += 0.5 * (lin_acc[i - 1] + lin_acc[i]) * dti

    speed = float(np.linalg.norm(v))
    result["release_speed"] = speed

    if speed < MIN_VALID_SPEED or speed > MAX_VALID_SPEED:
        result["status"] = "invalid_speed"
        return result

    expected_distance = DIST_COEFF * (speed ** 2) + DIST_BIAS
    result["expected_distance"] = float(expected_distance)
    result["strength"] = classify_throw(speed)
    return result


def print_result(result):
    print("\n=== THROW RESULT ===")
    print(f"Status:              {result['status']}")
    print(f"Sample count:        {result['sample_count']}")
    print(f"Median dt (s):       {result['median_dt']:.6f}")
    print(f"Release idx:         {result['release_idx']}")
    print(f"Release time (s):    {result['release_time']:.6f}")
    print(f"Release speed (m/s): {result['release_speed']:.6f}")
    print(f"Peak accel (m/s^2):  {result['peak_acc']:.6f}")
    print(f"Peak gyro (rad/s):   {result['peak_gyro']:.6f}")
    print(f"Expected dist (m):   {result['expected_distance']:.6f}")
    print(f"Strength:            {result['strength']}")


def run_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT}")

    recording = False
    samples = []

    while True:
        data, addr = sock.recvfrom(1024)
        line = data.decode("utf-8", errors="ignore").strip()

        if not line:
            continue

        if line == "START":
            recording = True
            samples = []
            print(f"\nSTART from {addr[0]}")
            continue

        if line == "END":
            if recording:
                result = process_throw(samples)
                print_result(result)
                write_result(result)
            recording = False
            samples = []
            continue

        if recording:
            sample = parse_sample_line(line)
            if sample is not None:
                samples.append(sample)


if __name__ == "__main__":
    run_listener()
