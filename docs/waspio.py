"""
This module provides simple IO for WASP frames over serial.

To get help with the command line interface run:
    python3 waspio.py -h

The relevant read and write functions for use with the REPL are:
    read_frame() & write_frame()

cc 03/09/21
"""

import struct
import argparse
from dataclasses import dataclass, field
from typing import Union, List
import serial
import crc8


DEFAULT_OUTPUT = bytes.fromhex('14000000ff000000030000000100000001020334')  # Simple size 1 data frame


@dataclass
class Frame:
    """Handles low level byte operations for interacting with WASP frames"""
    cmd: int
    data: Union[bytearray, List[int]]
    num_items: int = field(init=False)
    item_size: int = field(init=False)
    checksum: int = field(init=False, default=0)
    frame_len: int = field(init=False, default=0)
    header_bytes: bytes = field(init=False, default_factory=bytes)
    data_bytes: bytes = field(init=False, default_factory=bytes)
    frame_bytes: bytes = field(init=False, default_factory=bytes)

    def __post_init__(self):
        """Initializes derived values for instance"""
        self.num_items = len(self.data)

        if type(self.data) == list:
            self.item_size = 1
            data_pack_str = f"B"
        else:
            self.item_size = 4
            data_pack_str = f"i"

        self.frame_len = 4 * 4 + self.num_items * self.item_size + 1

        self.header_bytes = struct.pack("iiii", self.frame_len, self.cmd, self.num_items, self.item_size)
        self.data_bytes = struct.pack(f"{self.num_items}{data_pack_str}", *self.data)

        self.checksum = int.from_bytes(self.get_checksum(), "little")

        self.frame_bytes = self.header_bytes + self.data_bytes + bytes([self.checksum])

    def get_checksum(self):
        """Updates byte fields and returns CRC-8 hash"""
        self.header_bytes = struct.pack("<iiii", self.frame_len, self.cmd, self.num_items, self.item_size)
        self.data_bytes = struct.pack(f"<{self.num_items}{'B' if self.item_size == 1 else 'i'}", *self.data)

        hasher = crc8.crc8()
        hasher.update(self.header_bytes + self.data_bytes)
        return hasher.digest()

    @classmethod
    def from_bytes(cls, in_frame: bytes):
        """Parses a Frame object from a string of bytes"""
        frame_len, cmd, num_items, item_size = struct.unpack_from("<iiii", in_frame[:16])

        if item_size == 1:
            data = [b for b in in_frame[16:-1]]
        else:
            data = [b for b in in_frame[16:-1:4]]

        checksum = in_frame[-1]

        ret_obj = cls(cmd=cmd, data=data)

        if ret_obj.checksum != checksum:
            return None
        else:
            return ret_obj

    def __str__(self):
        ret_str = f"frame length: {self.frame_len}\n"
        ret_str += f"command: {self.cmd}\n"
        ret_str += f"num items: {self.num_items}\n"
        ret_str += f"item size: {self.item_size}\n"
        ret_str += f"data: {str(self.data)}\n"
        ret_str += f"checksum: {self.checksum}"
        return ret_str


def write_frame(port: str, baud: int = 115200, timeout: int = 0.1, frame: Frame = None) -> int:
    """Write frame to serial port, return num bytes written"""
    if frame is None:
        frame = Frame.from_bytes(DEFAULT_OUTPUT)

    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)

    try:
        ser.open()
    except serial.serialutil.SerialException:
        pass

    num_out = ser.write(frame.frame_bytes)
    return num_out


def read_frame(port: str, baud: int = 115200, timeout: int = 0.1) -> Frame:
    """Read frame from serial port, return Frame object"""
    ser = serial.Serial(port=port, baudrate=baud, timeout=None)  # Block until first input

    try:
        ser.open()
    except serial.serialutil.SerialException:
        pass

    frame_len_b = ser.read(4)  # Read frame length raw bytes
    frame_len = int.from_bytes(frame_len_b, "little")

    ser.timeout = timeout  # Set to desired timeout

    read_bytes = ser.read(frame_len - 1)

    frame = Frame.from_bytes(frame_len_b + read_bytes)

    return frame


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send or receive WASP frames over serial")

    parser.add_argument('direction', metavar='direction', type=str, choices=['read', 'write'],
                        help="Read from or write to serial")
    parser.add_argument('-b', '--baud', metavar='BAUD', type=int, help="The baud rate to operate with", default=115200)
    parser.add_argument('-t', '--timeout', metavar='TIMEOUT', type=float, help="Serial IO timeout", default=0.1)
    parser.add_argument('-p', '--port', metavar='PORT', type=str, help="The serial port to work with", required=True)

    args = parser.parse_args()

    if args.direction == "read":
        try:
            frame = read_frame(args.port, args.baud, args.timeout)
            print(f"\nReceived {frame.frame_len} byte frame:\n{frame}\n")
        except serial.SerialException as e:
            print(f"Serial read comms error: {str(e)}")
            exit(1)
    else:
        try:
            sent = write_frame(args.port, args.baud, args.timeout)
            print(f"Sent {sent or '0'} bytes")
        except serial.SerialException as e:
            print(f"Serial write comms error: {str(e)}")
            exit(1)
    exit(0)
