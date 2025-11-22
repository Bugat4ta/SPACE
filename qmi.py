# qmi8658.py
# Minimal MicroPython QMI8658 driver (read accel+gyro)
# Assumes I2C object passed in. Auto-detects address 0x6A/0x6B.
# Converts accel -> g, gyro -> deg/s using common sensitivity factors.
# Adapt scale_factors if you configure different FSR in device.

from time import sleep_ms

class QMI8658:
    # sensor data registers from QMI8658 datasheet
    REG_ACCEL_X_L = 0x35  # AX_L .. AZ_H (0x35 .. 0x3A)
    REG_GYRO_X_L  = 0x3B  # GX_L .. GZ_H (0x3B .. 0x40)

    # control registers (per Waveshare demos / datasheet)
    REG_CTRL5 = 0x06
    REG_CTRL6 = 0x07
    REG_CTRL7 = 0x08

    # possible I2C 7-bit addresses
    ADDR_CANDIDATES = (0x6A, 0x6B)  # depends on SA0/SDO pin

    def __init__(self, i2c, address=None, accel_lsb_per_g=16384.0, gyro_lsb_per_dps=16.0):
        """
        i2c: machine.I2C instance
        address: optional 7-bit i2c addr (if None, driver tries 0x6A/0x6B)
        accel_lsb_per_g: LSB per g (default assumes ±2g -> 16384)
        gyro_lsb_per_dps: LSB per dps (default assumes ±2048 dps -> 16)
        """
        self.i2c = i2c
        self.addr = address
        self.accel_lsb = float(accel_lsb_per_g)
        self.gyro_lsb = float(gyro_lsb_per_dps)

        if self.addr is None:
            self.addr = self._detect_address()
            if self.addr is None:
                raise OSError("QMI8658 not found on I2C (tried 0x6A/0x6B)")

        # Basic init sequence (per simple demos)
        # disable motion-on-demand etc, then enable accel+gyro
        try:
            # CTRL5 : accel/gyro LPF setup (safe default)
            self._write_reg(self.REG_CTRL5, 0x11)  # enable accel LPF (example from demos)
            # CTRL6 : disable motion-on-demand
            self._write_reg(self.REG_CTRL6, 0x00)
            # CTRL7 : enable accel+gyro (bits0/1 -> accel+gyro)
            self._write_reg(self.REG_CTRL7, 0x03)
            sleep_ms(10)
        except Exception:
            # If writes fail, still allow reads to try (user may handle)
            pass

    def _detect_address(self):
        for a in self.ADDR_CANDIDATES:
            try:
                if len(self.i2c.readfrom(a, 1)) >= 0:
                    return a
            except Exception:
                pass
        return None

    def _write_reg(self, reg, val):
        # single byte write
        self.i2c.writeto(self.addr, bytes([reg, val]))

    def _read_regs(self, reg, length):
        # some MicroPython I2C devices want a reg write then read
        # do writeto then readfrom into a buffer
        self.i2c.writeto(self.addr, bytes([reg]))
        return self.i2c.readfrom(self.addr, length)

    @staticmethod
    def _to_int16(high, low):
        v = (high << 8) | low
        if v & 0x8000:
            v = -((~v & 0xFFFF) + 1)
        return v

    def read_raw(self):
        """
        Read raw accel/gyro registers (12 bytes).
        Returns tuple: (ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)
        """
        data = self._read_regs(self.REG_ACCEL_X_L, 12)  # 0x35..0x40
        if len(data) != 12:
            raise OSError("QMI8658 read failed (expected 12 bytes)")
        ax = self._to_int16(data[1], data[0])  # high, low? datasheet: _H,_L; here bytes ordering adapted
        ay = self._to_int16(data[3], data[2])
        az = self._to_int16(data[5], data[4])
        gx = self._to_int16(data[7], data[6])
        gy = self._to_int16(data[9], data[8])
        gz = self._to_int16(data[11], data[10])
        return ax, ay, az, gx, gy, gz

    def get_accel_data(self):
        """
        Returns accel in g as a dict {'x': ax_g, 'y': ay_g, 'z': az_g}
        """
        ax, ay, az, gx, gy, gz = self.read_raw()
        return {'x': ax / self.accel_lsb, 'y': ay / self.accel_lsb, 'z': az / self.accel_lsb}

    def get_gyro_data(self):
        """
        Returns gyro in degrees per second as a dict {'x': gx_dps, 'y': gy_dps, 'z': gz_dps}
        """
        ax, ay, az, gx, gy, gz = self.read_raw()
        return {'x': gx / self.gyro_lsb, 'y': gy / self.gyro_lsb, 'z': gz / self.gyro_lsb}

    def read(self):
        """ convenience: return accel+gyro as tuple (ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps) """
        ax, ay, az, gx, gy, gz = self.read_raw()
        return (ax / self.accel_lsb, ay / self.accel_lsb, az / self.accel_lsb,
                gx / self.gyro_lsb, gy / self.gyro_lsb, gz / self.gyro_lsb)
