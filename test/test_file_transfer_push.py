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
import os
import math
import sys
import argparse
import serial
import time
import struct
from typing import List, Tuple

from file_transfer_payloads import (
    CMD_FILE_DATA,
    CMD_FILE_DATA_REQUEST,
    CMD_FILE_TRANSFER_DONE,
    FILE_DATA_REQUEST_STRUCT,
    build_file_package_payload,
)

# ============================================================================
# Protocol Constants
# ============================================================================

START_FLAG = 0x7E
END_FLAG = 0x7D
ESCAPE = 0xAA
PACKAGE_LEN = 128
POLYNOMIAL = 0xEDB88320
CRC32_TABLE = [
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
]

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

    def decode_bytes(self, data: bytes) -> Tuple[List[dict], int]:
        """
        Finds frames in a byte stream and returns them along with the number of bytes consumed.

        Returns:
            - A list of parsed frame dictionaries.
            - The index in the data buffer after the last processed frame.
        """
        frames = []
        i = 0
        while i < len(data):
            start_index = data.find(START_FLAG, i)
            if start_index == -1:
                # No start flag found, consume the rest of the buffer
                return frames, len(data)

            end_index = data.find(END_FLAG, start_index + 1)
            if end_index == -1:
                # Incomplete frame, consume up to the start flag and wait for more data
                return frames, start_index

            # Found a potential frame
            frame_content = data[start_index + 1:end_index]
            frame = self._parse_frame(bytes(frame_content))
            frames.append(frame)
            i = end_index + 1

        return frames, i

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


def crc32_file(file_path):
    """Calculate the cumulative CRC32 of a file."""
    crc = 0xFFFFFFFF
    with open(file_path, 'rb') as f:
        while chunk := f.read(1024 * 64):  # Read in 64 KB chunks
            for b in chunk:
                crc = (crc >> 8) ^ CRC32_TABLE[(crc ^ b) & 0xFF]
    return crc ^ 0xFFFFFFFF


def calculate_package_count(file_path, package_size):
    """
    Calculate the number of packages for a file given a package size.

    Args:
        file_path (str): Path to the input file.
        package_size (int): Size of each package in bytes.

    Returns:
        int: Number of packages required to cover the file.
    """
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

    file_size = os.path.getsize(file_path)
    # Calculate number of packages (round up if last package is partial)
    num_packages = math.ceil(file_size / package_size)
    return num_packages


def read_file_data(file_path, offset, length):
    """
    Read a portion of a file from a given offset.

    Args:
        file_path (str): Path to the file.
        offset (int): Start position in bytes.
        length (int): Number of bytes to read.

    Returns:
        bytes: Data read from the file.
    """
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

    file_size = os.path.getsize(file_path)

    if offset > file_size:
        return b''

    with open(file_path, "rb") as f:
        f.seek(offset)
        data = f.read(length)

    return data

def create_package_struct(buffer_size):
    # Corresponds to: uint8_t cmd; uint32_t seqId; uint16_t len; uint8_t data[buffer_size];
    return struct.Struct(f'<BIH{buffer_size}s')


# ============================================================================
# Main
# ============================================================================
def main():

    parser = argparse.ArgumentParser(
        description="Send framed data to Arduino and read response"
    )

    parser.add_argument("--port", required=True)
    parser.add_argument("--file", required=True)
    parser.add_argument("--size", type=int, default=128)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    codec = FrameCodec()

    total_packages = calculate_package_count(args.file, args.size)
    cumulative_crc = crc32_file(args.file)
    file_package_data = build_file_package_payload(args.file, total_packages, cumulative_crc)
    frame = codec.encode_frame(file_package_data)

    PACKAGE_STRUCT = create_package_struct(PACKAGE_LEN)

    # print(f"total package count {total_packages}")
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

    finished = False

    print("\n[SEND] ", " ".join(f"{b:02X}" for b in frame))
    ser.write(frame)
    ser.flush()

    rx = bytearray()
    try:
        while not finished:
            # print("[RECV] Listening...")

            # Read until an end flag is received or timeout
            line = ser.read_until(bytes([END_FLAG]))

            if not line:
                print("\nNo response received within timeout")
                break

            rx.extend(line)

            if args.verbose:
                print("RX:", " ".join(f"{b:02X}" for b in line))

            # print("\nRaw RX Buffer ({} bytes):".format(len(rx)))
            # print(" ", " ".join(f"{b:02X}" for b in rx))

            frames, consumed = codec.decode_bytes(rx)

            if consumed > 0:
                # print(f"  INFO: Consumed {consumed} bytes from buffer.")
                rx = rx[consumed:]

            if not frames:
                print("  WARNING: No complete frames were decoded, holding buffer for next read.")
                continue

            for i, f in enumerate(frames):
                if "error" in f:
                    print(f"  ERROR in frame {i}: {f['error']}")
                    continue

                if not f.get("crc_valid"):
                    print(f"  ERROR in frame {i}: Invalid CRC")
                    continue

                payload = f["payload"]
                try:
                    cmd, req_id = FILE_DATA_REQUEST_STRUCT.unpack(payload)
                    if cmd == CMD_FILE_DATA_REQUEST:
                        print(f"Transferring {req_id}/{total_packages}")
                        bufData = read_file_data(args.file, req_id*PACKAGE_LEN, PACKAGE_LEN)
                        package_data = PACKAGE_STRUCT.pack(CMD_FILE_DATA, req_id, len(bufData), bufData)
                        frame = codec.encode_frame(package_data)
                        # print("\n[SEND] ", " ".join(f"{b:02X}" for b in frame))
                        ser.write(frame)
                        ser.flush()
                    elif cmd == CMD_FILE_TRANSFER_DONE:
                        print(f"file transfer finished, status {req_id}")
                        finished = True
                    else:
                        print(f"  WARNING: Unhandled command 0x{cmd:02X} in frame {i}")
                except struct.error as e:
                    print(f"  ERROR unpacking payload in frame {i}: {e}")
                    continue
    finally:
        ser.close()
        print("\nDisconnected")


if __name__ == "__main__":
    main()
