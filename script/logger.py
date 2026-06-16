import serial
import struct
import csv
import time
from datetime import datetime

# ── 설정 ──────────────────────────────────────────────
PORT  = "COM3"    # 본인 포트로 변경 (예: "COM5", "/dev/ttyACM0")
BAUD  = 230400
OUTPUT = f"ads1220_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ── 프레임 상수 ───────────────────────────────────────
SOF0, SOF1 = 0xAA, 0x55
FRAME_LEN  = 21   # SOF(2) + seq(2) + enc0(4) + enc1(4) + lc0(4) + lc1(4) + checksum(1)


def xor_checksum(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def sync_to_frame(ser: serial.Serial) -> None:
    """0xAA 0x55 헤더를 찾을 때까지 1바이트씩 읽어 전진."""
    while True:
        if ser.read(1) == b'\xaa':
            if ser.read(1) == b'\x55':
                return


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser, \
         open(OUTPUT, "w", newline="") as f:

        writer = csv.writer(f)
        writer.writerow(["time_s", "seq", "enc0", "enc1", "lc0", "lc1"])

        print(f"Logging → {OUTPUT}    (Ctrl+C to stop)")

        t_start    = time.perf_counter()
        count      = 0
        bad_frames = 0

        while True:
            # 매 루프마다 SOF(AA 55)를 찾은 뒤 나머지 19바이트를 읽음
            sync_to_frame(ser)
            rest = ser.read(FRAME_LEN - 2)
            if len(rest) < FRAME_LEN - 2:
                continue

            frame = bytes([SOF0, SOF1]) + rest

            # 체크섬 검증
            if xor_checksum(frame[:20]) != frame[20]:
                bad_frames += 1
                continue

            # 파싱: little-endian
            seq                  = struct.unpack_from("<H", frame, 2)[0]
            enc0, enc1, lc0, lc1 = struct.unpack_from("<iiii", frame, 4)

            t = time.perf_counter() - t_start
            writer.writerow([f"{t:.6f}", seq, enc0, enc1, lc0, lc1])

            count += 1
            if count % 2000 == 0:
                hz = count / t if t > 0 else 0
                print(f"  {count:>8} frames  {hz:>7.1f} Hz  bad={bad_frames}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
