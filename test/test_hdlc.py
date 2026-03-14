#!/usr/bin/env python3
"""
Custom Frame Protocol Test Script

Frame format:

[START] [LEN_H][LEN_L] [PAYLOAD] [CRC_H][CRC_L] [END]

START : 0x7E
END   : 0x7D
ESC   : 0xAA

Byte Stuffing (payload + CRC only):
0xAA -> 0xAA 0x00
0x7E -> 0xAA 0x01
0x7D -> 0xAA 0x02
"""

import sys
import argparse
import serial
import time
from typing import List

# ============================================================================
# Protocol Constants
# ============================================================================

START_FLAG = 0x7E
END_FLAG = 0x7D
ESCAPE = 0xAA

# ============================================================================
# Protocol Codec
# ============================================================================


class FrameCodec:

    # ------------------------------------------------------------------------
    # CRC16 (matches C implementation exactly)
    # ------------------------------------------------------------------------

    @staticmethod
    def calc_crc16(data: bytes) -> int:
        crc = 0xFFFF

        for b in data:
            crc ^= (b << 8)

            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF

        return crc

    # ------------------------------------------------------------------------
    # Byte stuffing
    # ------------------------------------------------------------------------

    def stuff_bytes(self, data: bytes) -> bytes:

        out = bytearray()

        for b in data:

            if b == 0xAA:
                out.extend([0xAA, 0x00])

            elif b == 0x7E:
                out.extend([0xAA, 0x01])

            elif b == 0x7D:
                out.extend([0xAA, 0x02])

            else:
                out.append(b)

        return bytes(out)

    # ------------------------------------------------------------------------
    # Byte unstuffing
    # ------------------------------------------------------------------------

    def unstuff_bytes(self, data: bytes) -> bytes:

        out = bytearray()
        i = 0

        while i < len(data):

            if data[i] == ESCAPE:

                code = data[i + 1]

                if code == 0x00:
                    out.append(0xAA)

                elif code == 0x01:
                    out.append(0x7E)

                elif code == 0x02:
                    out.append(0x7D)

                else:
                    raise ValueError("Invalid escape sequence")

                i += 2

            else:
                out.append(data[i])
                i += 1

        return bytes(out)

    # ------------------------------------------------------------------------
    # Encode frame
    # ------------------------------------------------------------------------

    def encode_frame(self, payload: bytes) -> bytes:

        length = len(payload)
        length_bytes = length.to_bytes(2, "big")

        crc = self.calc_crc16(payload)
        crc_bytes = crc.to_bytes(2, "big")

        stuffed = self.stuff_bytes(payload + crc_bytes)

        frame = bytearray()

        frame.append(START_FLAG)
        frame.extend(length_bytes)
        frame.extend(stuffed)
        frame.append(END_FLAG)

        return bytes(frame)

    # ------------------------------------------------------------------------
    # Decode byte stream
    # ------------------------------------------------------------------------

    def decode_bytes(self, data: bytes) -> List[dict]:

        frames = []

        buffer = bytearray()
        inside = False

        for byte in data:

            if byte == START_FLAG:
                buffer.clear()
                inside = True
                continue

            if byte == END_FLAG and inside:

                frame = self._parse_frame(bytes(buffer))
                frames.append(frame)

                inside = False
                buffer.clear()
                continue

            if inside:
                buffer.append(byte)

        return frames

    # ------------------------------------------------------------------------
    # Parse frame
    # ------------------------------------------------------------------------

    def _parse_frame(self, frame_data: bytes) -> dict:

        if len(frame_data) < 4:
            return {"error": "Frame too short"}

        length = int.from_bytes(frame_data[0:2], "big")

        stuffed = frame_data[2:]

        try:
            data = self.unstuff_bytes(stuffed)
        except Exception as e:
            return {"error": str(e)}

        payload = data[:-2]
        recv_crc = int.from_bytes(data[-2:], "big")

        calc_crc = self.calc_crc16(payload)

        if len(payload) != length:
            return {"error": "Length mismatch"}

        return {
            "payload": payload,
            "crc": recv_crc,
            "crc_valid": recv_crc == calc_crc,
        }


# ============================================================================
# Utilities
# ============================================================================


def is_hex_byte(s: str) -> bool:

    try:
        v = int(s, 16)
        return 0 <= v <= 255
    except ValueError:
        return False


def parse_hex_string(text: str) -> bytes:

    parts = text.split()

    if all(is_hex_byte(p) for p in parts):
        return bytes(int(p, 16) for p in parts)

    return text.encode("utf-8")


# ============================================================================
# Main
# ============================================================================


def main():

    parser = argparse.ArgumentParser(
        description="Send framed data to Arduino and read response"
    )

    parser.add_argument("--port", required=True)
    parser.add_argument("--data", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    payload = parse_hex_string(args.data)

    print("Payload:")
    print(" ", " ".join(f"{b:02X}" for b in payload))

    codec = FrameCodec()

    frame = codec.encode_frame(payload)

    print("\nEncoded Frame:")
    print(" ", " ".join(f"{b:02X}" for b in frame))

    # ---------------------------------------------------------------------
    # Serial connection
    # ---------------------------------------------------------------------

    try:

        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=args.timeout
        )

        time.sleep(2)

        print(f"\nConnected to {args.port}")

    except serial.SerialException as e:

        print("Serial error:", e)
        sys.exit(1)

    try:

        print("\n[SEND]")
        ser.write(frame)
        ser.flush()

        start = time.time()
        rx = bytearray()

        print("[RECV] Listening...")

        while True:

            if time.time() - start > args.timeout:
                break

            if ser.in_waiting > 0:

                chunk = ser.read(ser.in_waiting)
                rx.extend(chunk)

                if args.verbose:
                    print("RX:", " ".join(f"{b:02X}" for b in chunk))

        if rx:

            print("\nRaw RX:")
            print(" ", " ".join(f"{b:02X}" for b in rx))

            frames = codec.decode_bytes(rx)

            for i, f in enumerate(frames):

                print(f"\nFrame {i+1}")

                if "error" in f:

                    print("  ERROR:", f["error"])

                else:

                    payload = f["payload"]

                    print("  Payload:", " ".join(f"{b:02X}" for b in payload))

                    try:
                        print("  Text:", payload.decode("utf-8"))
                    except:
                        pass

                    print(f"  CRC: 0x{f['crc']:04X}")
                    print("  CRC OK:", f["crc_valid"])

        else:

            print("\nNo response received")

    finally:

        ser.close()
        print("\nDisconnected")


if __name__ == "__main__":
    main()