import smbus2
import time

bus = smbus2.SMBus(1)

# 1. Wake up, select clock
bus.write_byte_data(0x68, 0x6B, 0x00)   # PWR_MGMT_1 — clear sleep
time.sleep(0.1)

# 2. Disable I2C master mode (must be off before bypass)
bus.write_byte_data(0x68, 0x6A, 0x00)   # USER_CTRL — I2C_MST_EN = 0
time.sleep(0.05)

# 3. Enable bypass
bus.write_byte_data(0x68, 0x37, 0x02)   # INT_PIN_CFG — BYPASS_EN = 1
time.sleep(0.1)

# 4. Verify registers
user_ctrl = bus.read_byte_data(0x68, 0x6A)
int_cfg   = bus.read_byte_data(0x68, 0x37)
print(f'USER_CTRL:   0x{user_ctrl:02X}  (want 0x00)')
print(f'INT_PIN_CFG: 0x{int_cfg:02X}  (want 0x02)')

# 5. Try AK8963
time.sleep(0.1)
try:
    who = bus.read_byte_data(0x0C, 0x00)
    print(f'AK8963 WHO_AM_I: 0x{who:02X}')
except OSError as e:
    print(f'AK8963 error: {e}')

bus.close()
