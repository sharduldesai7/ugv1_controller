#!/usr/bin/env python3
"""
MPU9250 Standalone Test Script
================================
Run this BEFORE the ROS2 node to verify:
  1. I2C is enabled and the device is detected
  2. WHO_AM_I register returns expected value
  3. All sensors are reading reasonable values
  4. AK8963 magnetometer is accessible via bypass

Usage:
    python3 test_imu.py
    python3 test_imu.py --bus 1 --addr 0x68 --calibrate

Does NOT require ROS2 — only needs smbus2.
"""

import sys
import time
import argparse
import math

try:
    import smbus2
except ImportError:
    print('ERROR: smbus2 not installed.')
    print('Run: pip3 install smbus2 --break-system-packages')
    sys.exit(1)


def check_i2c_device(bus_num: int, address: int) -> bool:
    """Check if a device responds at the given address."""
    try:
        bus = smbus2.SMBus(bus_num)
        bus.read_byte(address)
        bus.close()
        return True
    except OSError:
        return False


def scan_i2c(bus_num: int):
    """Scan all I2C addresses and print detected devices."""
    print(f'\nScanning I2C bus {bus_num}...')
    bus = smbus2.SMBus(bus_num)
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError:
            pass
    bus.close()

    if found:
        print(f'Found devices at: {[f"0x{a:02X}" for a in found]}')
    else:
        print('No I2C devices found.')
    return found


def main():
    parser = argparse.ArgumentParser(description='MPU9250 I2C test')
    parser.add_argument('--bus',       type=int, default=1,    help='I2C bus number')
    parser.add_argument('--addr',      type=lambda x: int(x, 0), default=0x68, help='MPU9250 address')
    parser.add_argument('--calibrate', action='store_true',    help='Run gyro + accel calibration')
    parser.add_argument('--samples',   type=int, default=50,   help='Number of samples to print')
    args = parser.parse_args()

    print('=' * 55)
    print('  MPU9250 Connection Test')
    print('=' * 55)

    # ── Step 1: Check I2C is enabled ──────────────────────────────────────────
    print(f'\n[1] Checking I2C bus {args.bus}...')
    try:
        bus = smbus2.SMBus(args.bus)
        print(f'    OK — /dev/i2c-{args.bus} opened')
    except FileNotFoundError:
        print(f'    FAIL — /dev/i2c-{args.bus} not found')
        print('    Enable I2C: sudo raspi-config → Interface Options → I2C')
        print('    Or add "dtparam=i2c_arm=on" to /boot/firmware/config.txt')
        sys.exit(1)

    # ── Step 2: Scan and detect ───────────────────────────────────────────────
    print(f'\n[2] Scanning bus...')
    found = scan_i2c(args.bus)

    if args.addr not in found:
        print(f'\n    FAIL — MPU9250 not found at 0x{args.addr:02X}')
        print('    Check wiring:')
        print('      VCC → 3.3V (Pin 1)')
        print('      GND → GND  (Pin 6)')
        print('      SDA → GPIO2 (Pin 3)')
        print('      SCL → GPIO3 (Pin 5)')
        print('      AD0 → GND for address 0x68, VCC for 0x69')
        sys.exit(1)
    else:
        print(f'\n    OK — MPU9250 detected at 0x{args.addr:02X}')

    # ── Step 3: WHO_AM_I check ────────────────────────────────────────────────
    print(f'\n[3] WHO_AM_I register check...')
    who = bus.read_byte_data(args.addr, 0x75)
    print(f'    Read: 0x{who:02X}', end='  ')
    if who in (0x71, 0x73, 0x70):
        print('OK — genuine MPU9250 (or compatible clone)')
    elif who == 0x68:
        print('WARN — this looks like an MPU6050 (no magnetometer)')
    else:
        print(f'WARN — unexpected value (may be a clone, continuing anyway)')

    # ── Step 4: Wake up and init ──────────────────────────────────────────────
    print(f'\n[4] Initialising sensor...')
    # Wake up
    bus.write_byte_data(args.addr, 0x6B, 0x01)   # PWR_MGMT_1: clock PLL
    time.sleep(0.1)
    bus.write_byte_data(args.addr, 0x6C, 0x00)   # Enable all axes
    bus.write_byte_data(args.addr, 0x1A, 0x03)   # DLPF 41Hz
    bus.write_byte_data(args.addr, 0x1B, 0x08)   # Gyro ±500 dps
    bus.write_byte_data(args.addr, 0x1C, 0x08)   # Accel ±4g
    bus.write_byte_data(args.addr, 0x37, 0x02)   # Enable bypass for AK8963
    time.sleep(0.05)
    print('    OK')

    # ── Step 5: Check AK8963 magnetometer ─────────────────────────────────────
    print(f'\n[5] AK8963 magnetometer check (address 0x0C)...')
    ak_found = 0x0C in found
    if ak_found:
        ak_who = bus.read_byte_data(0x0C, 0x00)
        print(f'    AK8963 WHO_AM_I: 0x{ak_who:02X}', end='  ')
        print('OK' if ak_who == 0x48 else 'WARN — unexpected value')
    else:
        print('    NOT FOUND — check INT_PIN_CFG bypass is enabled')
        print('    Note: magnetometer bypass may need a reboot to activate')

    # ── Step 6: Read raw values ───────────────────────────────────────────────
    print(f'\n[6] Reading {args.samples} samples...\n')

    import struct

    GYRO_SCALE  = (1.0 / 65.5) * (math.pi / 180)   # ±500dps → rad/s
    ACCEL_SCALE = (1.0 / 8192) * 9.80665            # ±4g    → m/s²

    print(f'{"Sample":>6}  {"Ax(m/s²)":>9} {"Ay":>9} {"Az":>9}  '
          f'{"Gx(r/s)":>9} {"Gy":>9} {"Gz":>9}  {"Temp(°C)":>9}')
    print('-' * 85)

    for i in range(args.samples):
        # Read accel (6 bytes from 0x3B)
        a = bus.read_i2c_block_data(args.addr, 0x3B, 6)
        ax = struct.unpack('>h', bytes([a[0], a[1]]))[0] * ACCEL_SCALE
        ay = struct.unpack('>h', bytes([a[2], a[3]]))[0] * ACCEL_SCALE
        az = struct.unpack('>h', bytes([a[4], a[5]]))[0] * ACCEL_SCALE

        # Read temperature (2 bytes from 0x41)
        t = bus.read_i2c_block_data(args.addr, 0x41, 2)
        temp = struct.unpack('>h', bytes([t[0], t[1]]))[0] / 333.87 + 21.0

        # Read gyro (6 bytes from 0x43)
        g = bus.read_i2c_block_data(args.addr, 0x43, 6)
        gx = struct.unpack('>h', bytes([g[0], g[1]]))[0] * GYRO_SCALE
        gy = struct.unpack('>h', bytes([g[2], g[3]]))[0] * GYRO_SCALE
        gz = struct.unpack('>h', bytes([g[4], g[5]]))[0] * GYRO_SCALE

        print(f'{i+1:>6}  {ax:>9.4f} {ay:>9.4f} {az:>9.4f}  '
              f'{gx:>9.5f} {gy:>9.5f} {gz:>9.5f}  {temp:>9.2f}')
        time.sleep(0.05)

    bus.close()

    # ── Summary ───────────────────────────────────────────────────────────────
    print('\n' + '=' * 55)
    print('  Summary')
    print('=' * 55)
    print(f'  Accel Z should be ~9.81 m/s² when flat (gravity)')
    print(f'  Gyro X/Y/Z should be ~0 when stationary')
    print(f'  Temperature should be ~25-40°C (chip temp)')
    print()
    print('  If values look good, run the ROS2 node:')
    print('    ros2 launch rover_imu imu.launch.py')
    print()
    print('  To save calibration biases to the launch file,')
    print('  run calibration via:')
    print('    ros2 service call /imu/calibrate_gyro std_srvs/srv/Trigger')


if __name__ == '__main__':
    main()
