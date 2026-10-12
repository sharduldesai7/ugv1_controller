#!/usr/bin/env python3
"""
RPLidar A1M8 health check (no ROS needed).

Repeatedly sends the GET_HEALTH request straight to the LiDAR over its serial
port and reports whether it answers. Useful for:
  - checking the LiDAR before launching the driver
  - finding an intermittent connection (flex the cables while it runs and
    watch for replies, or the lack of them)

A healthy unit replies with the 7-byte descriptor a5 5a 03 00 00 00 06
followed by 3 data bytes: status (0 good, 1 warning, 2 error) and a 16-bit
error code.

Usage:
  python3 scripts/lidar_health_check.py
  python3 scripts/lidar_health_check.py --port /dev/ttyUSB0 --duration 40
  python3 scripts/lidar_health_check.py --motor

Exit code:
  0  every request got a valid reply
  1  some requests were answered and some were not (intermittent link)
  2  no valid reply at all
  3  the serial port could not be opened

Stop the rplidar driver first: only one program can use the port at a time.
"""

import argparse
import sys
import time

import serial

GET_HEALTH = b'\xA5\x52'
DESCRIPTOR = bytes.fromhex('a55a0300000006')
STATUS_NAMES = {0: 'good', 1: 'warning', 2: 'error'}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send GET_HEALTH to the RPLidar and report the replies.')
    parser.add_argument('--port', default='/dev/rplidar',
                        help='serial port (default: /dev/rplidar)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='baud rate (default: 115200, the A1M8 rate)')
    parser.add_argument('--duration', type=float, default=10.0,
                        help='how long to run, in seconds (default: 10)')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='seconds between requests (default: 0.5)')
    parser.add_argument('--motor', action='store_true',
                        help='spin the motor while checking (DTR low)')
    return parser.parse_args()


def decode_reply(reply):
    """Return (ok, text) for a GET_HEALTH reply."""
    if len(reply) < len(DESCRIPTOR) + 3 or not reply.startswith(DESCRIPTOR):
        return False, 'unexpected reply: %s' % reply.hex()
    status = reply[len(DESCRIPTOR)]
    error_code = int.from_bytes(
        reply[len(DESCRIPTOR) + 1:len(DESCRIPTOR) + 3], 'little')
    name = STATUS_NAMES.get(status, 'unknown (%d)' % status)
    return True, 'status %s, error code 0x%04x' % (name, error_code)


def main():
    args = parse_args()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.3)
    except (serial.SerialException, OSError) as exc:
        print('Could not open %s: %s' % (args.port, exc), file=sys.stderr)
        return 3

    # On the A1 adapter board DTR low spins the motor, DTR high stops it
    port.dtr = not args.motor

    sent = 0
    replied = 0
    mid_line = False   # True while a row of dots is being printed
    start = time.time()
    print('Checking %s at %d baud for %.0f s (. = no reply)'
          % (args.port, args.baud, args.duration))

    try:
        while time.time() - start < args.duration:
            port.reset_input_buffer()
            port.write(GET_HEALTH)
            sent += 1
            reply = port.read(len(DESCRIPTOR) + 3)
            if reply:
                ok, text = decode_reply(reply)
                if ok:
                    replied += 1
                print('%s[%5.1fs] %s' % ('\n' if mid_line else '',
                                         time.time() - start, text), flush=True)
                mid_line = False
            else:
                print('.', end='', flush=True)
                mid_line = True
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()

    print('%s%d of %d requests answered'
          % ('\n' if mid_line else '', replied, sent))
    if replied == 0:
        print('No reply: check the cable between the LiDAR and its adapter '
              'board, the port name and that no other program is using it.')
        return 2
    if replied < sent:
        print('Intermittent: some requests were answered and some were not. '
              'Suspect a loose connector.')
        return 1
    print('LiDAR is answering reliably.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
