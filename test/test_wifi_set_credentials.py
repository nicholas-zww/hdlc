#!/usr/bin/env python3
"""
Send APP_COMMAND_SET_WIFI_CREDENTIALS (0x30) using the project frame protocol.

Command payload format:
  [0x30][ssid_len][password_len][ssid_bytes][password_bytes]
"""

import argparse
import sys
import time
from typing import List

import serial


START_FLAG = 0x7E
END_FLAG = 0x7D
ESCAPE = 0xAA

APP_COMMAND_SET_WIFI_CREDENTIALS = 0x30
MAX_SSID_LEN = 32
MAX_PASSWORD_LEN = 64


class FrameCodec:
    @staticmethod
    def calc_crc16(data: bytes) -> int:
        crc = 0xFFFF
        for value in data:
            crc ^= (value << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def stuff_bytes(data: bytes) -> bytes:
        out = bytearray()
        for value in data:
            if value == 0xAA:
                out.extend([0xAA, 0x00])
            elif value == 0x7E:
                out.extend([0xAA, 0x01])
            elif value == 0x7D:
                out.extend([0xAA, 0x02])
            else:
                out.append(value)
        return bytes(out)

    @staticmethod
    def unstuff_bytes(data: bytes) -> bytes:
        out = bytearray()
        index = 0
        while index < len(data):
            if data[index] == ESCAPE:
                if index + 1 >= len(data):
                    raise ValueError("incomplete escape sequence")
                code = data[index + 1]
                if code == 0x00:
                    out.append(0xAA)
                elif code == 0x01:
                    out.append(0x7E)
                elif code == 0x02:
                    out.append(0x7D)
                else:
                    raise ValueError(f"invalid escape code: 0x{code:02X}")
                index += 2
            else:
                out.append(data[index])
                index += 1
        return bytes(out)

    def encode_frame(self, payload: bytes) -> bytes:
        length = len(payload).to_bytes(2, "big")
        crc = self.calc_crc16(payload).to_bytes(2, "big")
        stuffed = self.stuff_bytes(payload + crc)
        return bytes([START_FLAG]) + length + stuffed + bytes([END_FLAG])

    def decode_bytes(self, data: bytes) -> List[dict]:
        frames = []
        collecting = False
        buffer = bytearray()
        for value in data:
            if value == START_FLAG:
                collecting = True
                buffer.clear()
                continue
            if value == END_FLAG and collecting:
                frames.append(self._parse_frame(bytes(buffer)))
                collecting = False
                buffer.clear()
                continue
            if collecting:
                buffer.append(value)
        return frames

    def _parse_frame(self, frame_data: bytes) -> dict:
        if len(frame_data) < 4:
            return {"error": "frame too short"}

        payload_length = int.from_bytes(frame_data[0:2], "big")
        stuffed = frame_data[2:]
        try:
            unstuffed = self.unstuff_bytes(stuffed)
        except Exception as exc:
            return {"error": str(exc)}

        if len(unstuffed) < 2:
            return {"error": "unstuffed data too short"}

        payload = unstuffed[:-2]
        crc_recv = int.from_bytes(unstuffed[-2:], "big")
        crc_calc = self.calc_crc16(payload)

        if len(payload) != payload_length:
            return {"error": "payload length mismatch"}

        return {"payload": payload, "crc_valid": crc_recv == crc_calc, "crc": crc_recv}


def build_command_payload(ssid: str, password: str) -> bytes:
    ssid_bytes = ssid.encode("utf-8")
    password_bytes = password.encode("utf-8")

    if len(ssid_bytes) > MAX_SSID_LEN:
        raise ValueError(f"SSID too long ({len(ssid_bytes)} bytes, max {MAX_SSID_LEN})")
    if len(password_bytes) > MAX_PASSWORD_LEN:
        raise ValueError(f"Password too long ({len(password_bytes)} bytes, max {MAX_PASSWORD_LEN})")

    return bytes(
        [APP_COMMAND_SET_WIFI_CREDENTIALS, len(ssid_bytes), len(password_bytes)]
    ) + ssid_bytes + password_bytes


def main() -> int:
    parser = argparse.ArgumentParser(description="Send WIFI credentials command to device")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbmodemXXXX")
    parser.add_argument("--ssid", required=True, help="WiFi SSID")
    parser.add_argument("--password", required=True, help="WiFi password")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    parser.add_argument("--timeout", type=float, default=3.0, help="Read timeout seconds after send")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose RX")
    args = parser.parse_args()

    try:
        cmd_payload = build_command_payload(args.ssid, args.password)
    except ValueError as exc:
        print(f"Invalid input: {exc}")
        return 2

    codec = FrameCodec()
    frame = codec.encode_frame(cmd_payload)

    print("Command Payload:")
    print(" ", " ".join(f"{b:02X}" for b in cmd_payload))
    print("\nEncoded Frame:")
    print(" ", " ".join(f"{b:02X}" for b in frame))

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 1

    try:
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        print("\nSent WIFI credentials command.")

        rx = bytearray()
        end_time = time.time() + args.timeout
        while time.time() < end_time:
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(waiting)
                rx.extend(chunk)
                if args.verbose:
                    print("RX:", " ".join(f"{b:02X}" for b in chunk))
            time.sleep(0.02)

        if not rx:
            print("No response frames observed (this can be normal).")
            return 0

        print("\nRaw RX:")
        print(" ", " ".join(f"{b:02X}" for b in rx))
        frames = codec.decode_bytes(bytes(rx))
        if not frames:
            print("No valid framed response found in RX stream.")
            return 0

        for index, frame_info in enumerate(frames, start=1):
            print(f"\nFrame {index}:")
            if "error" in frame_info:
                print(f"  ERROR: {frame_info['error']}")
                continue
            payload = frame_info["payload"]
            print("  Payload:", " ".join(f"{b:02X}" for b in payload))
            print(f"  CRC: 0x{frame_info['crc']:04X}")
            print(f"  CRC OK: {frame_info['crc_valid']}")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

