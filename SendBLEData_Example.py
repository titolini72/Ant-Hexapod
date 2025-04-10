import asyncio
import struct
import time
import random
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "ESP32C3-BLE Server"
WRITE_UUID = "96df8658-427c-458b-86de-0f6703f28977"  # Characteristic UUID
NOTIFY_UUID = "96df8658-427c-458b-86de-0f6703f28978"  # ACK Characteristic UUID

ACK_PREFIX = 0xAC

SOF1 = 0xAA
SOF2 = 0x55
CMD_SERVO_FRAME = 0x01
TIMESTAMP_MASK = 0xFFFFFFFF
PAYLOAD_OFFSET = 10

class RTTTracker:
    def __init__(self, alpha=0.2, margin_ms=3, min_floor_ms=20):
        self.alpha = alpha
        self.margin_ms = margin_ms
        self.min_floor_ms = min_floor_ms

        self.rtt_ema = None
        self.rtt_max = 0

    def update(self, rtt_ms):
        # EMA (smooth average)
        if self.rtt_ema is None:
            self.rtt_ema = rtt_ms
        else:
            self.rtt_ema = self.alpha * rtt_ms + (1 - self.alpha) * self.rtt_ema

        # Track peak (with slow decay)
        self.rtt_max = max(self.rtt_max * 0.95, rtt_ms)

    def get_floor(self):
        return max(self.min_floor_ms, self.rtt_max + self.margin_ms)

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
    if len(angles) != 22:
        raise ValueError("Need exactly 22 angles")
    if not all(0 <= a <= 180 for a in angles):
        raise ValueError("Angle out of range")
    if not (0 <= seq <= 255):
        raise ValueError("seq must be 0..255")
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000) & TIMESTAMP_MASK
    if not (0 <= timestamp_ms <= TIMESTAMP_MASK):
        raise ValueError("timestamp_ms must fit in uint32")

    payload = struct.pack("<H22B", wait_ms, *angles)  # little-endian
    payload_len = len(payload)

    frame_wo_crc = struct.pack("<BBBIBH", SOF1, SOF2, seq, timestamp_ms, CMD_SERVO_FRAME, payload_len) + payload
    # CRC over: seq + timestamp + type + length + payload
    crc = crc16_ccitt(frame_wo_crc[2:])
    frame = frame_wo_crc + struct.pack("<H", crc)

    return frame

def extract_wait_ms(frame: bytes) -> int:
    # payload starts after: SOF1 SOF2 SEQ TIMESTAMP(4) CMD LEN(2)
    wait_ms = struct.unpack_from("<H", frame, PAYLOAD_OFFSET)[0]
    return wait_ms

async def main():
    print("Scanning BLE devices...")
    devices = await BleakScanner.discover()
    device = next((d for d in devices if d.name == DEVICE_NAME), None)

    if device is None:
        print("Device not found")
        return

    seq = 16
    wait_ms = 500
    angles = [90] * 22
    frame = build_frame(seq, wait_ms, angles)

    ack_event = asyncio.Event()
    ack_result = {"ok": False, "seq": None, "status": None}

    def on_ack(sender, data: bytearray):
        # Expected: [ACK_PREFIX][seq][status][extra...]
        if len(data) >= 3 and data[0] == ACK_PREFIX:
            seq = data[1]
            status = data[2]

            extra = data[3:] if len(data) > 3 else b""

            ack_result["seq"] = seq
            ack_result["status"] = status
            ack_result["ok"] = (status == 0x00)
            ack_result["extra"] = extra

            ack_event.set()

    async with BleakClient(device.address) as client:
        print("Connected")
        await client.start_notify(NOTIFY_UUID, on_ack)

        # Example frames
        angles = [90] * 22
        sequence = 0

        rtt_tracker = RTTTracker()

        async def send_frame(frame, seq):
            wait_ms = extract_wait_ms(frame)

            ack_event.clear()

            t_send = time.perf_counter()

            # 🚀 IMPORTANT: response=False for low latency
            await client.write_gatt_char(WRITE_UUID, frame, response=False)

            # wait for ACK
            try:
                await asyncio.wait_for(ack_event.wait(), timeout=1.0)
                t_ack = time.perf_counter()

                if ack_result["seq"] != seq:
                    print(f"Wrong ACK seq (expected {seq}, got {ack_result['seq']})")
                    return False

                if ack_result["status"] != 0x00:
                    print(f"ACK error status={ack_result['status']}")
                    return False

                rtt_ms = (t_ack - t_send) * 1000.0

                adaptive_floor = rtt_tracker.get_floor()

                # Final delay = max(frame wait, adaptive floor)
                target_delay = max(wait_ms, adaptive_floor)

                remaining = (target_delay / 1000.0) - (t_ack - t_send)

                # Enforce frame timing
                remaining = (wait_ms / 1000.0) - (t_ack - t_send)
                if remaining > 0:
                    await asyncio.sleep(remaining)

                print(f"SEQ={seq:03d} RTT={rtt_ms:.1f} ms wait={wait_ms} ms floor={adaptive_floor:.1f} ms")

                return True

            except asyncio.TimeoutError:
                print(f"Timeout seq={seq}")
                return False

        # ================= SEND LOOP =================
        init = time.perf_counter()
        for i in range(20):
            wait_ms = 1000
            frame = build_frame(sequence, wait_ms, angles)
            delta = (time.perf_counter() - init )* 1000.
            print(f"T={delta:.2f}")
            ok = await send_frame(frame, sequence)
            if not ok:
                print("Stopping transmission")
                break

            sequence = (sequence + 1) & 0xFF

        await client.stop_notify(NOTIFY_UUID)


if __name__ == "__main__":
    asyncio.run(main())