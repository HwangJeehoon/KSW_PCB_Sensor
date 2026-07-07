import serial
import struct
import csv
import time
from datetime import datetime

# ── 설정 ──────────────────────────────────────────────
PORT   = "COM3"    # 본인 포트로 변경 (예: "COM5", "/dev/ttyACM0")
BAUD   = 230400
OUTPUT = f"ads1220_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ── 프레임 상수 ───────────────────────────────────────
SOF0, SOF1 = 0xAA, 0x55
# SOF(2) + t_us(4) + seq(2) + enc0(4) + enc1(4) + lc0(4) + lc1(4) + checksum(1) = 25 bytes
FRAME_LEN  = 25


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


def unwrap_micros(t_us_raw: int, prev_us: int, wrap_offset: int) -> tuple[int, int]:
    """micros() uint32 롤오버(~71.6분마다) 처리. 보정된 누적 us와 새 wrap_offset 반환."""
    WRAP = 1 << 32
    if prev_us != -1 and t_us_raw < (prev_us % WRAP) - 0x8000_0000:
        wrap_offset += WRAP
    return t_us_raw + wrap_offset, wrap_offset


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser, \
         open(OUTPUT, "w", newline="") as f:

        writer = csv.writer(f)
        writer.writerow(["time_s", "seq", "enc0", "enc1", "lc0", "lc1"])

        print(f"Logging → {OUTPUT}    (Ctrl+C to stop)")

        count      = 0
        bad_frames = 0
        t_us_start = -1   # Arduino micros() 기준 시각
        prev_raw   = -1   # 롤오버 감지용 직전 raw 값
        wrap_offset = 0

        while True:
            sync_to_frame(ser)
            rest = ser.read(FRAME_LEN - 2)
            if len(rest) < FRAME_LEN - 2:
                continue

            frame = bytes([SOF0, SOF1]) + rest

            if xor_checksum(frame[:24]) != frame[24]:
                bad_frames += 1
                continue

            # 파싱: little-endian
            # f[2..5]=t_us, f[6..7]=seq, f[8..23]=enc0,enc1,lc0,lc1
            t_us_raw             = struct.unpack_from("<I",    frame, 2)[0]
            seq                  = struct.unpack_from("<H",    frame, 6)[0]
            enc0, enc1, lc0, lc1 = struct.unpack_from("<iiii", frame, 8)

            # 롤오버 보정
            t_us_abs, wrap_offset = unwrap_micros(t_us_raw, prev_raw, wrap_offset)
            prev_raw = t_us_raw

            if t_us_start < 0:
                t_us_start = t_us_abs

            t_s = (t_us_abs - t_us_start) * 1e-6

            writer.writerow([f"{t_s:.6f}", seq, enc0, enc1, lc0, lc1])

            count += 1
            if count % 2000 == 0:
                print(f"  {count:>8} frames  {t_s:>8.2f} s  bad={bad_frames}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
