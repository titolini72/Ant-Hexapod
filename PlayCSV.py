"""
Play a servo-angle CSV (e.g. WalkAndHead.csv) to the hexapod over BLE.

The CSV must have a header row. The first column is a per-frame time/interval
value and the remaining 22 columns are the servo angles (0..180) in the same
order the firmware expects:

    Time, LF_Femur, LF_Tibia, LF_Feet, LM_Femur, ... Head_Roll, Head_Pitch,
    Head_Grip, Tail

Each data row is sent as one servo frame. Use --loop to replay continuously.

Examples:
    python PlayCSV.py WalkAndHead.csv
    python PlayCSV.py WalkAndHead.csv --loop
    python PlayCSV.py WalkAndHead.csv --loop --count 5 --wait-ms 40
"""

import argparse
import asyncio
import csv
import struct
import time

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32C3-BLE Server"
WRITE_UUID = "96df8658-427c-458b-86de-0f6703f28977"   # Characteristic UUID
NOTIFY_UUID = "96df8658-427c-458b-86de-0f6703f28978"  # ACK Characteristic UUID

ACK_PREFIX = 0xAC

SOF1 = 0xAA
SOF2 = 0x55
CMD_SERVO_FRAME = 0x01
TIMESTAMP_MASK = 0xFFFFFFFF

NUM_SERVOS = 22


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(seq, wait_ms, angles, timestamp_ms=None):
    if len(angles) != NUM_SERVOS:
        raise ValueError(f"Need exactly {NUM_SERVOS} angles, got {len(angles)}")
    if not all(0 <= a <= 180 for a in angles):
        raise ValueError("Angle out of range (0..180)")
    if not (0 <= seq <= 255):
        raise ValueError("seq must be 0..255")
    if not (0 <= wait_ms <= 0xFFFF):
        raise ValueError("wait_ms must fit in uint16")
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000) & TIMESTAMP_MASK

    payload = struct.pack(f"<H{NUM_SERVOS}B", wait_ms, *angles)  # little-endian
    payload_len = len(payload)

    frame_wo_crc = struct.pack(
        "<BBBIBH", SOF1, SOF2, seq, timestamp_ms, CMD_SERVO_FRAME, payload_len
    ) + payload
    # CRC over: seq + timestamp + type + length + payload
    crc = crc16_ccitt(frame_wo_crc[2:])
    return frame_wo_crc + struct.pack("<H", crc)


def load_csv(path):
    """Return a list of (time_value, [22 angles]) tuples from the CSV file."""
    frames = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("CSV file is empty")

        for line_no, row in enumerate(reader, start=2):
            if not row or all(cell.strip() == "" for cell in row):
                continue
            try:
                values = [int(float(cell)) for cell in row if cell.strip() != ""]
            except ValueError as exc:
                raise ValueError(f"Non-numeric data on line {line_no}: {exc}") from exc

            if len(values) < NUM_SERVOS + 1:
                raise ValueError(
                    f"Line {line_no} has {len(values)} columns, "
                    f"expected at least {NUM_SERVOS + 1}"
                )

            time_value = values[0]
            angles = values[1:1 + NUM_SERVOS]
            angles = [max(0, min(180, a)) for a in angles]
            frames.append((time_value, angles))

    if not frames:
        raise ValueError("CSV contained no data rows")
    return frames


async def play(args):
    print("Scanning BLE devices...")
    device = None
    if args.address:
        device = await BleakScanner.find_device_by_address(args.address)
    else:
        devices = await BleakScanner.discover()
        device = next((d for d in devices if d.name == args.device), None)

    if device is None:
        print("Device not found")
        return

    frames = load_csv(args.csv)
    print(f"Loaded {len(frames)} frames from {args.csv}")

    ack_event = asyncio.Event()
    ack_result = {"ok": False, "seq": None, "status": None}

    def on_ack(_sender, data: bytearray):
        if len(data) >= 3 and data[0] == ACK_PREFIX:
            ack_result["seq"] = data[1]
            ack_result["status"] = data[2]
            ack_result["ok"] = (data[2] == 0x00)
            ack_event.set()

    async with BleakClient(device.address) as client:
        print("Connected")
        await client.start_notify(NOTIFY_UUID, on_ack)

        seq = 0

        async def send_frame(frame, seq, wait_ms):
            ack_event.clear()
            t_send = time.perf_counter()
            await client.write_gatt_char(WRITE_UUID, frame, response=False)

            if args.no_ack:
                # Fire-and-forget pacing based purely on wait_ms.
                await asyncio.sleep(wait_ms / 1000.0)
                return True

            try:
                await asyncio.wait_for(ack_event.wait(), timeout=args.timeout)
            except asyncio.TimeoutError:
                print(f"Timeout seq={seq}")
                return False

            if ack_result["seq"] != seq:
                print(f"Wrong ACK seq (expected {seq}, got {ack_result['seq']})")
                return False
            if ack_result["status"] != 0x00:
                print(f"ACK error status={ack_result['status']}")
                return False

            elapsed = time.perf_counter() - t_send
            remaining = (wait_ms / 1000.0) - elapsed
            if remaining > 0:
                await asyncio.sleep(remaining)
            return True

        loop_index = 0
        try:
            while True:
                loop_index += 1
                t0 = time.perf_counter()
                for i, (time_value, angles) in enumerate(frames):
                    wait_ms = args.wait_ms if args.wait_ms is not None else time_value
                    wait_ms = max(0, min(0xFFFF, int(wait_ms)))

                    frame = build_frame(seq, wait_ms, angles)
                    ok = await send_frame(frame, seq, wait_ms)
                    if not ok:
                        print("Stopping transmission")
                        return
                    seq = (seq + 1) & 0xFF

                dt = time.perf_counter() - t0
                print(f"Loop {loop_index} done ({len(frames)} frames in {dt:.2f}s)")

                if not args.loop:
                    break
                if args.count and loop_index >= args.count:
                    break
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            await client.stop_notify(NOTIFY_UUID)


def parse_args():
    p = argparse.ArgumentParser(
        description="Play a servo-angle CSV to the hexapod over BLE."
    )
    p.add_argument("csv", help="Path to the CSV file (e.g. WalkAndHead.csv)")
    p.add_argument("--loop", action="store_true",
                   help="Replay the file continuously")
    p.add_argument("--count", type=int, default=0,
                   help="With --loop, stop after this many loops (0 = forever)")
    p.add_argument("--wait-ms", type=int, default=None,
                   help="Override per-frame wait in ms (default: use CSV time column)")
    p.add_argument("--device", default=DEVICE_NAME,
                   help="BLE device name to scan for")
    p.add_argument("--address", default=None,
                   help="Connect directly to this BLE address (skips name scan)")
    p.add_argument("--timeout", type=float, default=1.0,
                   help="ACK wait timeout in seconds")
    p.add_argument("--no-ack", action="store_true",
                   help="Do not wait for ACK; pace using wait-ms only")
    return p.parse_args()


if __name__ == "__main__":
    asyncio.run(play(parse_args()))
