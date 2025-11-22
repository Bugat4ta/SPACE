from machine import I2C, Pin
import time

# I2C setup
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)  # match your header pinout

QMI8658_ADDR = 0x6A  # default; try 0x6B if AD0 pin is high

# QMI8658 registers
WHO_AM_I = 0x00
CTRL7 = 0x08
AX_L = 0x35
GY_L = 0x3B
GY_H = 0x3C

# Helper functions
def read_reg(addr, reg, nbytes=1):
    return i2c.readfrom_mem(addr, reg, nbytes)

def write_reg(addr, reg, val):
    i2c.writeto_mem(addr, reg, bytes([val]))

# Scan I2C bus
devices = i2c.scan()
print("I2C devices found:", [hex(d) for d in devices])

if QMI8658_ADDR not in devices:
    print(f"QMI8658 not found at 0x{QMI8658_ADDR:X}")
else:
    # Enable accelerometer + gyro
    QMI8658_CTRL7_ACC_ENABLE = 0x01
    QMI8658_CTRL7_GYR_ENABLE = 0x02
    write_reg(QMI8658_ADDR, CTRL7, QMI8658_CTRL7_ACC_ENABLE | QMI8658_CTRL7_GYR_ENABLE)

    # Read WHO_AM_I
    who_am_i = read_reg(QMI8658_ADDR, WHO_AM_I)[0]
    print("QMI8658 WHO_AM_I:", hex(who_am_i))

    # Read accelerometer (X axis example)
    ax_l = read_reg(QMI8658_ADDR, 0x35)[0]
    ax_h = read_reg(QMI8658_ADDR, 0x36)[0]
    ax = (ax_h << 8) | ax_l
    if ax & 0x8000:  # 16-bit signed
        ax -= 0x10000
    print("Accelerometer X:", ax)

    # Read gyro X (same principle)
    gx_l = read_reg(QMI8658_ADDR, 0x3B)[0]
    gx_h = read_reg(QMI8658_ADDR, 0x3C)[0]
    gx = (gx_h << 8) | gx_l
    if gx & 0x8000:
        gx -= 0x10000
    print("Gyroscope X:", gx)
