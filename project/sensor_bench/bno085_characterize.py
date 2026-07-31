"""BNO085 bench characterization through the MCP2221A USB-I2C bridge.

Logs timestamped orientation (quaternion + Euler) to CSV and reports the
numbers that currently sit as placeholders in
motion_comp_sim/step2_corrupt_to_sensor.m:
  - effective update rate (Hz) and inter-sample jitter
  - static angular noise (deg RMS per axis) when held still
  - orientation drift over the capture window

Absolute end-to-end latency seen here also includes USB-HID bridge overhead,
so treat it as a rough upper bound only. Precise sensor latency gets measured
on the board later (PS I2C, hardware-timestamped).

Run (PowerShell, from this folder):
    $env:BLINKA_MCP2221 = "1"
    python bno085_characterize.py --seconds 120 --out bno085_static.csv

For a static-noise/drift run, hold the sensor perfectly still the whole time.
"""
import argparse
import csv
import math
import time

import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


def quat_to_euler_deg(qi, qj, qk, qr):
    """Return (roll, pitch, yaw) in degrees from a unit quaternion (ZYX)."""
    sinr_cosp = 2.0 * (qr * qi + qj * qk)
    cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (qr * qj - qk * qi)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def rms_std(vals):
    m = sum(vals) / len(vals)
    return (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--out", default="bno085_capture.csv")
    args = ap.parse_args()

    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    time.sleep(0.5)  # let the first reports arrive before timing

    rows = []
    intervals = []
    t0 = time.perf_counter()
    t_prev = None
    print(f"Capturing for {args.seconds:.0f} s ... hold still for a static run.")

    while time.perf_counter() - t0 < args.seconds:
        qi, qj, qk, qr = bno.quaternion
        t = time.perf_counter()
        roll, pitch, yaw = quat_to_euler_deg(qi, qj, qk, qr)
        rows.append((t - t0, qi, qj, qk, qr, roll, pitch, yaw))
        if t_prev is not None:
            intervals.append(t - t_prev)
        t_prev = t

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "qi", "qj", "qk", "qr", "roll_deg", "pitch_deg", "yaw_deg"])
        w.writerows(rows)

    n = len(rows)
    dur = rows[-1][0] if rows else 0.0
    hz = (n - 1) / dur if dur > 0 else 0.0
    mean_dt = sum(intervals) / len(intervals) if intervals else 0.0
    jit = rms_std(intervals) if intervals else 0.0

    rolls = [r[5] for r in rows]
    pitches = [r[6] for r in rows]
    yaws = [r[7] for r in rows]

    print(f"\nSamples: {n}   Duration: {dur:.2f} s")
    print(f"Update rate: {hz:.1f} Hz   mean dt: {mean_dt * 1e3:.2f} ms   "
          f"jitter (std): {jit * 1e3:.2f} ms")
    print("Static angular noise (deg RMS)  roll: %.4f  pitch: %.4f  yaw: %.4f"
          % (rms_std(rolls), rms_std(pitches), rms_std(yaws)))
    print("Drift over window (deg)  roll: %+.3f  pitch: %+.3f  yaw: %+.3f"
          % (rolls[-1] - rolls[0], pitches[-1] - pitches[0], yaws[-1] - yaws[0]))
    print(f"\nCSV written: {args.out}")


if __name__ == "__main__":
    main()
