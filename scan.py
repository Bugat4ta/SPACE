from machine import I2C, Pin
i2c = I2C(0, scl=Pin(17), sda=Pin(16))
print("Scanning I2C bus...")
devices = i2c.scan()
print("Found devices:", [hex(d) for d in devices])
