from machine import Pin, I2C

i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
devices = i2c.scan()
print("I2C devices found:", [hex(d) for d in devices])
