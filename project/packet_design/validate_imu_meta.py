"""Validate the imu_meta_t byte layout that rides in mic_packet_t.reserved[56].

Uses a REAL quaternion from the Jul 29 BNO085 capture, encodes it into the
56-byte reserved block exactly as the board firmware would (big-endian, Q14),
decodes it back as the host parser would, and checks the round-trip is within
Q14 quantization. Also verifies the full packet geometry (70B header + 1024B
payload = 1094B) so both ends stay in lockstep with the layout header.

Pure stdlib. Run:  python validate_imu_meta.py
"""
import csv
import os
import struct
import sys

# --- layout constants (mirror imu_meta_layout.h) ---
IMU_META_MARKER = 0x494D
IMU_QUAT_SCALE = 16384.0  # 2^14

# reserved[56] layout, big-endian:
#   marker(H) valid(B) accuracy(B) i(h) j(h) k(h) real(h) ts_us(I) seq(H) pad[38]
RESERVED_FMT = ">HBB hhhh I H 38x"
RESERVED_SIZE = struct.calcsize(RESERVED_FMT)

HEADER_BYTES = 70
SAMPLES_PER_MIC = 512
PAYLOAD_BYTES = SAMPLES_PER_MIC * 2
PACKET_BYTES = HEADER_BYTES + PAYLOAD_BYTES


def q14(x):
    """Encode a quaternion component to Q14 int16, saturating to the int16 range."""
    v = int(round(x * IMU_QUAT_SCALE))
    return max(-32768, min(32767, v))


def encode_reserved(qi, qj, qk, qr, ts_us, seq, accuracy=3, valid=1):
    block = struct.pack(
        RESERVED_FMT,
        IMU_META_MARKER, valid, accuracy,
        q14(qi), q14(qj), q14(qk), q14(qr),
        ts_us & 0xFFFFFFFF, seq & 0xFFFF,
    )
    assert len(block) == 56, f"reserved block is {len(block)} bytes, must be 56"
    return block


def decode_reserved(block):
    (marker, valid, accuracy, i, j, k, r, ts_us, seq) = struct.unpack(RESERVED_FMT, block)
    return {
        "marker": marker,
        "valid": valid,
        "accuracy": accuracy,
        "quat": (i / IMU_QUAT_SCALE, j / IMU_QUAT_SCALE,
                 k / IMU_QUAT_SCALE, r / IMU_QUAT_SCALE),
        "timestamp_us": ts_us,
        "seq": seq,
    }


def load_last_quaternion(csv_path):
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    last = rows[-1]
    return float(last["qi"]), float(last["qj"]), float(last["qk"]), float(last["qr"])


def main():
    # 1) geometry sanity
    assert RESERVED_SIZE == 56, f"format packs to {RESERVED_SIZE}, expected 56"
    assert HEADER_BYTES == 4 + 2 + 4 + 4 + 56, "header field sizes do not sum to 70"
    print(f"Packet geometry OK: header {HEADER_BYTES} + payload {PAYLOAD_BYTES} "
          f"= {PACKET_BYTES} bytes")

    # 2) real quaternion round-trip
    here = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(here, "..", "sensor_bench", "bno085_static.csv")
    qi, qj, qk, qr = load_last_quaternion(csv_path)
    print(f"\nSource quaternion (from bno085_static.csv):")
    print(f"  i={qi:+.6f}  j={qj:+.6f}  k={qk:+.6f}  real={qr:+.6f}")

    block = encode_reserved(qi, qj, qk, qr, ts_us=123456789, seq=42)
    print("\nreserved[56] hex:")
    print("  " + " ".join(f"{b:02x}" for b in block))

    dec = decode_reserved(block)
    print(f"\nDecoded: marker=0x{dec['marker']:04X} valid={dec['valid']} "
          f"accuracy={dec['accuracy']} seq={dec['seq']} ts_us={dec['timestamp_us']}")

    tol = 1.0 / IMU_QUAT_SCALE  # one Q14 quantum
    errs = [abs(a - b) for a, b in zip(dec["quat"], (qi, qj, qk, qr))]
    max_err = max(errs)
    print(f"  quat={tuple(round(v, 6) for v in dec['quat'])}")
    print(f"  max round-trip error: {max_err:.2e}  (Q14 quantum {tol:.2e})")

    ok = (dec["marker"] == IMU_META_MARKER and dec["valid"] == 1
          and dec["seq"] == 42 and dec["timestamp_us"] == 123456789
          and max_err <= tol)
    print("\nRESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
