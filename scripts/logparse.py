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
MAX_MSG_DATA_LENGTH = 128
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
    M(0x01): Spec("test", "<f", ["test_val"]),  
    M(0x02): Spec("canard_cmd", "d", ["cmd_angle"]),
    M(0x03): Spec("controller_input", "<L4xdddd", ["timestamp", "roll_angle", "roll_rate", "canard_coeff", "pressure_dynamic"]),
    M(0x04): Spec("movella", "<Ldddddddddf?",
    [
        "movella_time",
        "movella_acc_x", "movella_acc_y", "movella_acc_z",
        "movella_gyr_x", "movella_gyr_y", "movella_gyr_z",
        "movella_mag_x", "movella_mag_y", "movella_mag_z",
        "movella_bar",
        "movella_is_dead",
    ]),
    M(0x05): Spec("ekf_ctx", "<dddddddddddddd",
    [
        "attitude_w", "attitude_x", "attitude_y", "attitude_z",
        "rates_x", "rates_y", "rates_z",
        "velocity_x", "velocity_y", "velocity_z",
        "altitude", "CL", "delta", "t"
    ]),
    M(0x06): Spec("encoder", "<f", ["encoder_value"]),
    M(0x07): Spec("pololu", "<Ldddddddddf?",
    [
        "polulu_time",
        "polulu_acc_x", "polulu_acc_y", "polulu_acc_z",
        "polulu_gyr_x", "polulu_gyr_y", "polulu_gyr_z",
        "polulu_mag_x", "polulu_mag_y", "polulu_mag_z",
        "polulu_bar",
        "polulu_is_dead",
    ]),
    M(0x08): Spec("raw_pololu", "<hhhhhhhhhih",
    [
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z",
        "mag_x", "mag_y", "mag_z",
        "baro_pres", "baro_temp"
    ]),
    # Insert new types above this line in the format:
    # M(unique_small_integer): Spec(name, format, [field, ...]),
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
