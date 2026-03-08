#!/usr/bin/env python3
"""
HDLC Test Script for Arduino FreeRTOS Project

Sends a byte array packed in HDLC format to Arduino via serial port,
then reads responses until timeout.

Usage:
    python3 test_hdlc.py <port> <data> [--baud BAUD] [--timeout TIMEOUT]

Examples:
    python3 test_hdlc.py /dev/ttyUSB0 "Hello"
    python3 test_hdlc.py /dev/ttyUSB0 "Hello" --baud 115200 --timeout 5
    python3 test_hdlc.py COM3 "48 65 6C 6C 6F" --baud 115200
    python3 test_hdlc.py /dev/cu.usbserial "Test message" --timeout 10
"""

import sys
import argparse
import serial
import time
from typing import Optional, List

# ============================================================================
# HDLC Constants
# ============================================================================

HDLC_FLAG = 0x7E
HDLC_ESCAPE = 0x7D
HDLC_ESCAPE_MASK = 0x20

# ============================================================================
# HDLC Encoder/Decoder
# ============================================================================

class HDLCCodec:
    """HDLC frame encoder and decoder"""

    @staticmethod
    def calc_crc16(data: bytes) -> int:
        """Calculate CRC-16-CCITT"""
        crc = 0xFFFF
        table = [
            0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
            0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
            0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
            0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
            0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
            0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
            0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
            0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
            0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
            0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
            0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
            0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
            0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
            0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
            0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
            0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
            0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
            0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
            0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
            0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
            0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
            0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
            0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
            0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
            0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
            0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
            0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
            0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
            0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
            0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
            0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
            0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
        ]

        for byte in data:
            idx = (crc ^ byte) & 0xFF
            crc = (crc >> 8) ^ table[idx]

        return crc

    def encode_frame(self, address: int, control: int, data: bytes) -> bytes:
        """
        Encode data into an HDLC frame with bit stuffing

        Args:
            address: Frame address byte
            control: Frame control byte
            data: Data payload

        Returns:
            Complete HDLC frame (with flags)
        """
        # Build frame without flags
        frame_data = bytearray()
        frame_data.append(address)
        frame_data.append(control)
        frame_data.extend(data)

        # Calculate FCS
        fcs = self.calc_crc16(frame_data)
        frame_data.append(fcs & 0xFF)  # Low byte
        frame_data.append((fcs >> 8) & 0xFF)  # High byte

        # Apply bit stuffing (escape FLAG and ESCAPE bytes)
        stuffed = bytearray()
        for byte in frame_data:
            if byte == HDLC_FLAG or byte == HDLC_ESCAPE:
                stuffed.append(HDLC_ESCAPE)
                stuffed.append(byte ^ HDLC_ESCAPE_MASK)
            else:
                stuffed.append(byte)

        # Add flags
        frame = bytearray()
        frame.append(HDLC_FLAG)
        frame.extend(stuffed)
        frame.append(HDLC_FLAG)

        return bytes(frame)

    def decode_bytes(self, data: bytes) -> List[dict]:
        """
        Decode a stream of bytes into HDLC frames

        Args:
            data: Raw byte stream

        Returns:
            List of decoded frame dicts
        """
        frames = []
        rx_buffer = bytearray()
        escape_next = False

        for byte in data:
            # Handle escape
            if escape_next:
                escape_next = False
                byte ^= HDLC_ESCAPE_MASK
                rx_buffer.append(byte)
                continue

            # Check for escape character
            if byte == HDLC_ESCAPE:
                escape_next = True
                continue

            # Check for flag
            if byte == HDLC_FLAG:
                if len(rx_buffer) > 0:
                    # End of frame - try to parse
                    if len(rx_buffer) >= 4:  # Min: ADDR + CTRL + FCS(2)
                        frame = self._parse_frame(bytes(rx_buffer))
                        frames.append(frame)
                    rx_buffer.clear()
                continue

            # Regular data byte
            rx_buffer.append(byte)

        return frames

    def _parse_frame(self, frame_data: bytes) -> dict:
        """
        Parse a complete HDLC frame (without flags)

        Args:
            frame_data: Frame data without flags

        Returns:
            Dict with address, control, data, and CRC status
        """
        if len(frame_data) < 4:
            return {'error': 'Frame too short'}

        address = frame_data[0]
        control = frame_data[1]
        received_fcs = (frame_data[-1] << 8) | frame_data[-2]
        data = frame_data[2:-2]

        # Verify CRC
        calc_fcs = self.calc_crc16(frame_data[:-2])
        crc_valid = (received_fcs == calc_fcs)

        return {
            'address': address,
            'control': control,
            'data': data,
            'fcs': received_fcs,
            'crc_valid': crc_valid
        }


# ============================================================================
# Main Function
# ============================================================================

def parse_hex_string(hex_str: str) -> bytes:
    """
    Parse a hex string into bytes

    Args:
        hex_str: Space-separated hex values or plain string

    Returns:
        Bytes object
    """
    # Try to parse as hex string first
    parts = hex_str.split()
    if all(is_hex_byte(p) for p in parts):
        return bytes(int(p, 16) for p in parts)

    # Otherwise treat as plain string
    return hex_str.encode('utf-8')


def is_hex_byte(s: str) -> bool:
    """Check if string is a valid hex byte (00-FF)"""
    try:
        val = int(s, 16)
        return 0 <= val <= 255
    except ValueError:
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Send HDLC-framed data to Arduino and read response',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  %(prog)s /dev/ttyUSB0 "Hello"
  %(prog)s /dev/ttyUSB0 "Hello" --baud 115200 --timeout 5
  %(prog)s COM3 "48 65 6C 6C 6F" --baud 115200
  %(prog)s /dev/cu.usbserial "Test message" --timeout 10
        '''
    )

    parser.add_argument(
        '--port',
        help='Serial port (e.g., /dev/ttyUSB0, COM3, /dev/cu.usbserial)'
    )

    parser.add_argument(
        '--data',
        help='Data to send (plain text or space-separated hex bytes like "48 65 6C 6C 6F")'
    )

    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )

    parser.add_argument(
        '--timeout', '-t',
        type=float,
        default=5.0,
        help='Receive timeout in seconds (default: 5.0)'
    )

    parser.add_argument(
        '--address', '-a',
        type=int,
        default=0x01,
        help='HDLC frame address (default: 0x01)'
    )

    parser.add_argument(
        '--control', '-c',
        type=int,
        default=0x03,
        help='HDLC frame control byte (default: 0x03)'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Show verbose output including raw bytes'
    )

    args = parser.parse_args()

    # Parse data
    if is_hex_byte(args.data) or ' ' in args.data:
        data = parse_hex_string(args.data)
        print(f"Input: Hex bytes")
    else:
        data = args.data.encode('utf-8')
        print(f"Input: Text string")

    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Timeout: {args.timeout}s")
    print(f"Address: 0x{args.address:02X}")
    print(f"Control: 0x{args.control:02X}")
    print(f"Data bytes: {' '.join(f'0x{b:02X}' for b in data)}")

    # Create codec
    codec = HDLCCodec()

    # Encode frame
    frame = codec.encode_frame(args.address, args.control, data)

    print(f"HDLC Frame ({len(frame)} bytes):")
    print(f"  {' '.join(f'{b:02X}' for b in frame)}")
    print()

    # Open serial port
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=args.timeout
        )
        time.sleep(2)  # Wait for Arduino reset
        print(f"✓ Connected to {args.port}")
    except serial.SerialException as e:
        print(f"✗ Failed to open {args.port}: {e}")
        sys.exit(1)

    try:
        # Send frame
        print(f"[SEND] Transmitting HDLC frame...")
        ser.write(frame)
        ser.flush()
        print(f"[SEND] Done")
        print()

        # Receive response
        print(f"[RECV] Listening for {args.timeout}s...")
        start_time = time.time()
        rx_data = bytearray()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= args.timeout:
                print(f"[RECV] Timeout after {elapsed:.2f}s")
                break

            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                rx_data.extend(chunk)
                if args.verbose:
                    print(f"[RECV] Got {len(chunk)} bytes: {' '.join(f'{b:02X}' for b in chunk)}")

        # Decode received frames
        if len(rx_data) > 0:
            print(f"\n[RECV] Total received: {len(rx_data)} bytes")
            print(f"  Raw: {' '.join(f'{b:02X}' for b in rx_data)}")
            print()

            frames = codec.decode_bytes(rx_data)

            if frames:
                print(f"[RECV] Decoded {len(frames)} HDLC frame(s):")
                for i, frame in enumerate(frames):
                    if 'error' in frame:
                        print(f"  Frame {i+1}: ERROR - {frame['error']}")
                    else:
                        print(f"\n  Frame {i+1}:")
                        print(f"    Address: 0x{frame['address']:02X}")
                        print(f"    Control: 0x{frame['control']:02X}")
                        print(f"    Data: {frame['data']}")
                        try:
                            print(f"    Data (text): {frame['data'].decode('utf-8', errors='replace')}")
                        except:
                            pass
                        print(f"    FCS: 0x{frame['fcs']:04X}")
                        print(f"    CRC: {'✓ OK' if frame['crc_valid'] else '✗ FAIL'}")
            else:
                print("[RECV] No valid HDLC frames found")
        else:
            print("[RECV] No data received")

    finally:
        # Cleanup
        if ser.is_open:
            ser.close()
            print("\n✓ Disconnected")


if __name__ == '__main__':
    main()
