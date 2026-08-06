"""Send the Oradar MS200 command that stops its motor."""

import argparse
from functools import reduce
import operator
import time

import serial


def _deactivate_packet():
    # Oradar protocol fields: little-endian 0xF5A5 header, SET_RUN_MODE,
    # WRITE_PARAM, one-byte payload, inactive mode 0x80, XOR, 0x31F2 tail.
    payload = bytes((0xA5, 0xF5, 0xA2, 0xC1, 0x01, 0x80))
    checksum = reduce(operator.xor, payload, 0)
    return payload + bytes((checksum, 0x31, 0xF2))


def _arguments():
    parser = argparse.ArgumentParser(
        description='Stop the Oradar MS200 LiDAR motor.',
    )
    parser.add_argument('port', nargs='?', default='/dev/ttyAMA1')
    return parser.parse_args()


def main():
    args = _arguments()
    packet = _deactivate_packet()

    try:
        with serial.Serial(
            port=args.port,
            baudrate=230400,
            timeout=1.0,
            write_timeout=1.0,
        ) as lidar:
            lidar.reset_input_buffer()
            lidar.write(packet)
            lidar.flush()
            time.sleep(0.8)
    except (OSError, serial.SerialException) as exc:
        print(f'Failed to stop LiDAR on {args.port}: {exc}')
        return 1

    print(f'LiDAR motor-off command sent on {args.port}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
