#!/usr/bin/env python3
import argparse
import sys
import struct
from collections import namedtuple

"""
Special values from src/application/logger/log.h
This script won't work properly if these values are inconsistent with the logger header file!
"""

# Size of each message region in data buffers (bytes)
MAX_MSG_DATA_LENGTH = 64
# Magic number encoded into message type values
LOG_DATA_MAGIC = 0x4c44
# Macro used to encode magic value into message type values
def M(v): return ((v & 0xffff) << 16) | LOG_DATA_MAGIC

# Definition of a message type's name, struct.unpack format string, and corresponding list of struct field names
Spec = namedtuple("Spec", ["name", "format", "fields"])

"""
Definition of message types and their structs.
See https://docs.python.org/3/library/struct.html#format-strings for format string syntax.

format quick reference:
- Start string with '<' for little-endian byte order
- 'L' for uint32_t (unsigned long), 'l' for int32_t (long)
- 'f' for float, 'd' for double
"""
FORMATS = {
    0x44414548: Spec("header", "<LL", ["version", "index"]),
    # Insert new types above this line in the format:
    # M(unique_small_integer): Spec(name, format, [field, ...]),
    M(0x01): Spec("test", "<f", ["test_val"]),  
    M(0x02): Spec("controller", "f", ["cmd_angle"]),
    M(0x03): Spec("controller_output", "<fL", ["commanded_angle", "timestamp"]),
    M(0x04): Spec("imu_reading", "<LfffLfffLfff?b", ["polulu_timestamp","polulu_accel_x","polulu_accel_y","polulu_accel_z","polulu_gyro_x","polulu_gyro_y","polulu_gyro_z","polulu_mag_x","polulu_mag_y","polulu_mag_z","polulu_barometer","polulu_is_dead","movella_timestamp","movella_accel_x","movella_accel_y","movella_accel_z","movella_gyro_x","movella_gyro_y","movella_gyro_z","movella_mag_x","movella_mag_y","movella_mag_z","movella_barometer","movella_is_dead"]),
}

def parse_argv(argv):
    p = argparse.ArgumentParser()
    p.add_argument("infile")
    return p.parse_args(argv[1:])

def main(argv=None):
    args = parse_argv(argv or sys.argv)
    with open(args.infile, "rb") as f:
        data = f.read()
    # Loop through message regions
    for pos in range(0, len(data), MAX_MSG_DATA_LENGTH):
        type_int, timestamp = struct.unpack_from("<Lf", data, pos)
        print(f"[{timestamp:.1f}] ", end="")
        try:
            spec = FORMATS[type_int]
            print(f"{spec.name} (type ", end="")
            if spec.name != "header":
                print(type_int >> 16, end="")
            else:
                print("\"HEAD\"", end="")
            print(") with")
            values = struct.unpack_from(spec.format, data, pos + 8)
            for field, value in zip(spec.fields, values):
                print(f"    {field}: {value}")
        except KeyError:
            print(f"unknown type 0x{type_int:08x} with\n    unknown data:", data[pos + 8:pos + MAX_MSG_DATA_LENGTH])

if __name__ == "__main__":
    main()
