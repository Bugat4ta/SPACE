import time
import math
from machine import I2C, Pin

# ---------------- QMI8658 Driver ----------------
class QMI8658:
    def __init__(self, i2c, addr=None):
        self.i2c = i2c
        self.addr = addr if addr else 0x6A
        if self._whoami() != 0x05:
            raise RuntimeError("QMI8658 not found on I2C bus")
        self._init_sensor()

    def _whoami(self):
        return self._read_reg(0x00, 1)[0]

    def _init_sensor(self):
        self._write_reg(0x0C, 0x60)  # reset
        time.sleep(0.1)
        self._write_reg(0x08, 0x03)  # enable accel + gyro

    def _write_reg(self, reg, value):
        self.i2c.writeto(self.addr, bytes([reg, value]))

    def _read_reg(self, reg, nbytes=1):
        self.i2c.writeto(self.addr, bytes([reg]))
        return self.i2c.readfrom(self.addr, nbytes)

    def get_accel_data(self):
        data = self._read_reg(0x35, 6)
        x = self._twos_complement(data[1] << 8 | data[0], 16)
        y = self._twos_complement(data[3] << 8 | data[2], 16)
        z = self._twos_complement(data[5] << 8 | data[4], 16)
        scale = 2 / 32768  # ±2g
        return {'x': x*scale, 'y': y*scale, 'z': z*scale}

    def get_gyro_data(self):
        data = self._read_reg(0x3B, 6)
        x = self._twos_complement(data[1] << 8 | data[0], 16)
        y = self._twos_complement(data[3] << 8 | data[2], 16)
        z = self._twos_complement(data[5] << 8 | data[4], 16)
        scale = 250 / 32768  # ±250 dps
        return {'x': x*scale, 'y': y*scale, 'z': z*scale}

    @staticmethod
    def _twos_complement(val, bits):
        if val & (1 << (bits - 1)):
            val -= (1 << bits)
        return val

# ---------------- Helper Functions ----------------
def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def meters_per_deg_lat(lat_deg):
    return 111132.954 - 559.822 * math.cos(2*deg2rad(lat_deg)) + 1.175 * math.cos(4*deg2rad(lat_deg))

def meters_per_deg_lon(lat_deg):
    return (111412.84 * math.cos(deg2rad(lat_deg)) - 93.5 * math.cos(3*deg2rad(lat_deg)))

def compute_checksum(payload: str) -> str:
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"{c:02X}"

def body_to_nav(accel_body, roll_deg, pitch_deg):
    phi = deg2rad(roll_deg)
    theta = deg2rad(pitch_deg)
    sphi = math.sin(phi); cphi = math.cos(phi)
    stheta = math.sin(theta); ctheta = math.cos(theta)
    a_x = accel_body['x']
    a_y = accel_body['y']
    a_z = accel_body['z']
    a_n =  ctheta * a_x + sphi * stheta * a_y + cphi * stheta * a_z
    a_e =            cphi * a_y - sphi * a_z
    a_d = -stheta * a_x + sphi * ctheta * a_y + cphi * ctheta * a_z
    return {'n': a_n, 'e': a_e, 'd': a_d}

# ---------------- Filters ----------------
class KalmanFilter:
    def __init__(self, q=0.1, r=1.0, x0=0.0, p0=1.0):
        self.q = q
        self.r = r
        self.x = x0
        self.p = p0

    def update(self, z):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (z - self.x)
        self.p *= (1 - k)
        return self.x

class PID:
    def __init__(self, kp, ki, kd, output_limits=(-999, 999)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = None
        self.output_limits = output_limits

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral
        if self.last_error is None:
            d = 0.0
        else:
            d = self.kd * ((error - self.last_error) / dt)
        self.last_error = error
        out = p + i + d
        lo, hi = self.output_limits
        return max(min(out, hi), lo)

# ---------------- CanSat Variables ----------------
alpha_attitude = 0.98
dt_nominal = 0.01
start_lat = 34.0
start_lon = -117.0
m_per_deg_lat = meters_per_deg_lat(start_lat)
m_per_deg_lon = meters_per_deg_lon(start_lat)

pos_n = 0.0
pos_e = 0.0
vel_n = 0.0
vel_e = 0.0
pitch = 0.0
roll = 0.0
yaw = 0.0

kf_vn = KalmanFilter(q=0.05, r=0.5)
kf_ve = KalmanFilter(q=0.05, r=0.5)

last_baro_pressure = None
sea_level_pressure = 101325.0

CRASH_ACCEL_THRESHOLD = 35.0
crashed = False
last_altitude = None
altitude_stable_time = 0.0

mission_state = "BOOT"
waypoint_lat = start_lat + 0.001
waypoint_lon = start_lon + 0.001
waypoint_n = (waypoint_lat - start_lat) * m_per_deg_lat
waypoint_e = (waypoint_lon - start_lon) * m_per_deg_lon

pid_yaw = PID(kp=2.0, ki=0.05, kd=0.4, output_limits=(-5, 5))
packet_counter = 0

def build_telemetry_packet(ts, alt, vn, ve, lat, lon, pitch, yaw, state, crash, batt=3.9):
    global packet_counter
    packet_counter += 1
    payload = (
        f"CANSAT,{packet_counter},{ts:.2f},{lat:.6f},{lon:.6f},"
        f"{alt:.2f},{pitch:.2f},{yaw:.2f},{vn:.3f},{ve:.3f},"
        f"{batt:.2f},{state},{int(crash)}"
    )
    chk = compute_checksum(payload)
    return f"${payload}*{chk}"

def check_crash(accel, altitude, dt):
    global last_altitude, altitude_stable_time, crashed, mission_state
    mag = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
    if mag > CRASH_ACCEL_THRESHOLD:
        crashed = True
        mission_state = "CRASHED"
        return True
    if last_altitude is None:
        last_altitude = altitude
    if abs(altitude - last_altitude) < 0.2:
        altitude_stable_time += dt
    else:
        altitude_stable_time = 0.0
        last_altitude = altitude
    return False

# ---------------- I2C and Sensor ----------------
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
mpu = QMI8658(i2c)

# ---------------- Main Loop ----------------
def main_loop():
    global pos_n, pos_e, vel_n, vel_e, pitch, roll, yaw
    global crashed, mission_state

    last_time = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            dt = now - last_time
            if dt <= 0:
                dt = dt_nominal
            last_time = now

            try:
                accel = mpu.get_accel_data()
                gyro = mpu.get_gyro_data()
            except Exception as e:
                print("Sensor read error:", e)
                time.sleep(0.05)
                continue

            # Convert accel to m/s² if small numbers
            accel_m = {k: v*9.80665 for k,v in accel.items()} if max(abs(v) for v in accel.values()) < 20 else accel.copy()
            gyro_dps = gyro.copy()

            # Complementary filter
            accel_pitch = rad2deg(math.atan2(-accel_m['x'], math.sqrt(accel_m['y']**2 + accel_m['z']**2)))
            accel_roll  = rad2deg(math.atan2(accel_m['y'], accel_m['z']))
            pitch = alpha_attitude * (pitch + gyro_dps['x'] * dt) + (1-alpha_attitude) * accel_pitch
            roll  = alpha_attitude * (roll  + gyro_dps['y'] * dt) + (1-alpha_attitude) * accel_roll
            yaw   = yaw + gyro_dps['z'] * dt

            # Navigation acceleration
            a_nav = body_to_nav(accel_m, roll, pitch)
            a_nav['d'] -= 9.80665
            vel_n += a_nav['n'] * dt
            vel_e += a_nav['e'] * dt
            vel_n = kf_vn.update(vel_n)
            vel_e = kf_ve.update(vel_e)
            if not crashed:
                pos_n += vel_n * dt
                pos_e += vel_e * dt

            altitude = 0.0  # placeholder
            check_crash(accel_m, altitude, dt)

            # Mission state logic
            if mission_state == "BOOT":
                mission_state = "ASCENT"
            elif mission_state == "ASCENT" and altitude > 50.0:
                mission_state = "APOGEE"
            elif mission_state == "APOGEE" and altitude < 50.0:
                mission_state = "DEPLOY"
            elif mission_state == "DEPLOY":
                mission_state = "DESCENT"
            elif mission_state == "DESCENT":
                mission_state = "NAVIGATION"
            elif mission_state == "NAVIGATION":
                dx = waypoint_n - pos_n
                dy = waypoint_e - pos_e
                if math.sqrt(dx*dx + dy*dy) < 10.0:
                    mission_state = "WAYPOINT_REACHED"
            elif mission_state == "WAYPOINT_REACHED" and altitude < 2.0:
                mission_state = "LANDING"

            # Yaw control
            dx = waypoint_n - pos_n
            dy = waypoint_e - pos_e
            target_heading = rad2deg(math.atan2(dy, dx))
            yaw_error = target_heading - yaw
            while yaw_error > 180: yaw_error -= 360
            while yaw_error < -180: yaw_error += 360
            yaw += pid_yaw.compute(yaw_error, dt) * dt

            lat = start_lat + (pos_n / m_per_deg_lat)
            lon = start_lon + (pos_e / m_per_deg_lon)

            ts = time.time()
            packet = build_telemetry_packet(ts, altitude, vel_n, vel_e, lat, lon, pitch, yaw, mission_state, crashed)
            print(packet)

            time.sleep(max(0.0, dt_nominal - (time.monotonic() - now)))

            if mission_state == "LANDING":
                print("Mission complete: landing state reached.")
                break

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")
    except Exception as e:
        print("Fatal error:", e)

if __name__ == "__main__":
    main_loop()
