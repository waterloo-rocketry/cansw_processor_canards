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
    M(0x02): Spec("canard_cmd", "f", ["cmd_angle"]),
    M(0x03): Spec("controller_input", "<Lfffff", ["timestamp", "roll_angle", "roll_rate", "canard_angle", "canard_coeff", "pressure_dynamic"]),
    M(0x04): Spec("movella", "<Ldddddddddf?",
    [
        "movella_time",
        "movella_acc_x", "movella_acc_y", "movella_acc_z",
        "movella_gyr_x", "movella_gyr_y", "movella_gyr_z",
        "movella_mag_x", "movella_mag_y", "movella_mag_z",
        "movella_bar",
        "movella_is_dead",
    ]),
    M(0x05): Spec("x_state", "<ddddddddddddd",
    [
        "attitude_w", "attitude_x", "attitude_y", "attitude_z",
        "rates_x", "rates_y", "rates_z",
        "velocity_x", "velocity_y", "velocity_z",
        "altitude", "CL", "delta"
    ]),
    M(0x06): Spec("encoder", "<H", ["encoder_value"]),
    M(0x07): Spec("pololu", "<Ldddddddddf?",
    [
        "polulu_time",
        "polulu_acc_x", "polulu_acc_y", "polulu_acc_z",
        "polulu_gyr_x", "polulu_gyr_y", "polulu_gyr_z",
        "polulu_mag_x", "polulu_mag_y", "polulu_mag_z",
        "polulu_bar",
        "polulu_is_dead",
    ]),
    M(0x08): Spec("raw_pololu", "<HHHHHHHHHLH",
    [
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z",
        "mag_x", "mag_y", "mag_z",
        "baro_pressure", "baro_temp"
    ]
),

}

ACC_FS = 16.0 / 32767      # g / LSB
GYRO_FS = 2000.0 / 32767   # dps / LSB
MAG_FS = 16.0 / 32767      # gauss / LSB
BARO_FS = 100.0 / 4096.0   # Pa / LSB
TEMP_FS = 1.0 / 100.0      # °C / LSB

def int16(val):
    """Convert raw unsigned 16-bit to signed"""
    return val if val < 0x8000 else val - 0x10000

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
        spec = FORMATS[type_int]

        # convert pololu
        if type_int == M(0x08):
            raw_vals = struct.unpack_from(spec.format, data, pos + 8)

            acc = [int16(raw_vals[0]) * ACC_FS,
                int16(raw_vals[1]) * ACC_FS,
                int16(raw_vals[2]) * ACC_FS]

            gyro = [int16(raw_vals[3]) * GYRO_FS,
                    int16(raw_vals[4]) * GYRO_FS,
                    int16(raw_vals[5]) * GYRO_FS]

            mag = [int16(raw_vals[6]) * MAG_FS,
                int16(raw_vals[7]) * MAG_FS,
                int16(raw_vals[8]) * MAG_FS]

            baro = raw_vals[9] * BARO_FS
            temp = int16(raw_vals[10]) * TEMP_FS

            print(f"    acc_x: {acc[0]:.3f} g")
            print(f"    acc_y: {acc[1]:.3f} g")
            print(f"    acc_z: {acc[2]:.3f} g")
            print(f"    gyro_x: {gyro[0]:.2f} °/s")
            print(f"    gyro_y: {gyro[1]:.2f} °/s")
            print(f"    gyro_z: {gyro[2]:.2f} °/s")
            print(f"    mag_x: {mag[0]:.3f} G")
            print(f"    mag_y: {mag[1]:.3f} G")
            print(f"    mag_z: {mag[2]:.3f} G")
            print(f"    baro_pressure: {baro:.1f} Pa")
            print(f"    baro_temp: {temp:.2f} °C")

        try:
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
