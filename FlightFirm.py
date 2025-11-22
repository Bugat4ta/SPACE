from machine import I2C, Pin
import time
import math

# ---------------- I2C Setup ----------------
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)  # adjust for your board
QMI8658_ADDR = 0x6A  # try 0x6B if AD0 high

# ---------------- QMI8658 Registers ----------------
WHO_AM_I = 0x00
CTRL7 = 0x08
AX_L = 0x35
GY_L = 0x3B
GY_H = 0x3C

ACC_ENABLE = 0x01
GYR_ENABLE = 0x02

# ---------------- Helper Functions ----------------
def read_reg(addr, reg, nbytes=1):
    return i2c.readfrom_mem(addr, reg, nbytes)

def write_reg(addr, reg, val):
    i2c.writeto_mem(addr, reg, bytes([val]))

def twos_complement(val, bits=16):
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

# ---------------- QMI8658 Driver ----------------
class QMI8658:
    def __init__(self, addr=QMI8658_ADDR):
        self.addr = addr
        devices = i2c.scan()
        print("I2C devices found:", [hex(d) for d in devices])
        if addr not in devices:
            raise RuntimeError(f"QMI8658 not found at 0x{addr:X}")
        write_reg(addr, CTRL7, ACC_ENABLE | GYR_ENABLE)
        who_am_i = read_reg(addr, WHO_AM_I)[0]
        print("QMI8658 WHO_AM_I:", hex(who_am_i))

    def get_accel_data(self):
        x = twos_complement((read_reg(self.addr, AX_L+1)[0]<<8) | read_reg(self.addr, AX_L)[0]) / 1000.0
        y = twos_complement((read_reg(self.addr, AX_L+3)[0]<<8) | read_reg(self.addr, AX_L+2)[0]) / 1000.0
        z = twos_complement((read_reg(self.addr, AX_L+5)[0]<<8) | read_reg(self.addr, AX_L+4)[0]) / 1000.0
        return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self):
        x = twos_complement((read_reg(self.addr, GY_H)[0]<<8) | read_reg(self.addr, GY_L)[0]) / 16.4
        y = twos_complement((read_reg(self.addr, GY_H+2)[0]<<8) | read_reg(self.addr, GY_L+2)[0]) / 16.4
        z = twos_complement((read_reg(self.addr, GY_H+4)[0]<<8) | read_reg(self.addr, GY_L+4)[0]) / 16.4
        return {'x': x, 'y': y, 'z': z}

# ---------------- Helper Functions for CanSat ----------------
def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def meters_per_deg_lat(lat_deg):
    return 111132.954 - 559.822*math.cos(2*deg2rad(lat_deg)) + 1.175*math.cos(4*deg2rad(lat_deg))

def meters_per_deg_lon(lat_deg):
    return 111412.84*math.cos(deg2rad(lat_deg)) - 93.5*math.cos(3*deg2rad(lat_deg))

def compute_checksum(payload: str) -> str:
    c = 0
    for ch in payload: c ^= ord(ch)
    return f"{c:02X}"

class KalmanFilter:
    def __init__(self, q=0.1, r=1.0, x0=0.0, p0=1.0):
        self.q = q; self.r = r; self.x = x0; self.p = p0
    def update(self, z):
        self.p += self.q
        k = self.p/(self.p+self.r)
        self.x += k*(z - self.x)
        self.p *= (1 - k)
        return self.x

class PID:
    def __init__(self, kp, ki, kd, output_limits=(-999,999)):
        self.kp=kp; self.ki=ki; self.kd=kd
        self.integral=0; self.last_error=None
        self.output_limits=output_limits
    def compute(self, error, dt):
        if dt <= 0: return 0.0
        p = self.kp * error
        self.integral += error*dt
        i = self.ki * self.integral
        d = 0.0 if self.last_error is None else self.kd*((error - self.last_error)/dt)
        self.last_error = error
        lo, hi = self.output_limits
        return max(min(p+i+d, hi), lo)

def body_to_nav(accel_body, roll_deg, pitch_deg):
    phi = deg2rad(roll_deg); theta = deg2rad(pitch_deg)
    sphi, cphi = math.sin(phi), math.cos(phi)
    stheta, ctheta = math.sin(theta), math.cos(theta)
    a_x, a_y, a_z = accel_body['x'], accel_body['y'], accel_body['z']
    a_n = ctheta*a_x + sphi*stheta*a_y + cphi*stheta*a_z
    a_e = cphi*a_y - sphi*a_z
    a_d = -stheta*a_x + sphi*ctheta*a_y + cphi*ctheta*a_z
    return {'n': a_n, 'e': a_e, 'd': a_d}

def build_telemetry_packet(ts, alt, vn, ve, lat, lon, pitch, yaw, state, crash, batt=3.9):
    global packet_counter
    packet_counter += 1
    payload = (f"CANSAT,{packet_counter},{ts:.2f},{lat:.6f},{lon:.6f},"
               f"{alt:.2f},{pitch:.2f},{yaw:.2f},{vn:.3f},{ve:.3f},"
               f"{batt:.2f},{state},{int(crash)}")
    chk = compute_checksum(payload)
    return f"${payload}*{chk}"

def check_crash(accel, altitude, dt):
    global last_altitude, altitude_stable_time, crashed, mission_state
    mag = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
    if mag > CRASH_ACCEL_THRESHOLD:
        crashed = True
        mission_state = "CRASHED"
        return True
    if last_altitude is None: last_altitude = altitude
    if abs(altitude - last_altitude) < 0.2:
        altitude_stable_time += dt
    else:
        altitude_stable_time = 0
        last_altitude = altitude
    return False

# ---------------- Initialization ----------------
mpu = QMI8658()
alpha_attitude = 0.98
dt_nominal = 0.01
start_lat, start_lon = 34.0, -117.0
m_per_deg_lat = meters_per_deg_lat(start_lat)
m_per_deg_lon = meters_per_deg_lon(start_lat)
pos_n = pos_e = vel_n = vel_e = 0.0
kf_vn = KalmanFilter(q=0.05, r=0.5)
kf_ve = KalmanFilter(q=0.05, r=0.5)
pitch = roll = yaw = 0.0
CRASH_ACCEL_THRESHOLD = 35.0
crashed = False
last_altitude = None
altitude_stable_time = 0.0
mission_state = "BOOT"
waypoint_lat = start_lat + 0.001
waypoint_lon = start_lon + 0.001
waypoint_n = (waypoint_lat - start_lat) * m_per_deg_lat
waypoint_e = (waypoint_lon - start_lon) * m_per_deg_lon
pid_yaw = PID(2.0, 0.05, 0.4, (-5,5))
packet_counter = 0

# ---------------- Main Loop ----------------
def main_loop():
    global pos_n, pos_e, vel_n, vel_e, pitch, roll, yaw, crashed, mission_state
    last_time = time.ticks_ms()
    try:
        while True:
            now = time.ticks_ms()
            dt = time.ticks_diff(now, last_time)/1000.0
            dt = dt_nominal if dt <= 0 else dt
            last_time = now

            try:
                accel = mpu.get_accel_data()
                gyro = mpu.get_gyro_data()
            except Exception as e:
                print("Sensor read error:", e)
                time.sleep(0.05)
                continue

            accel_m = {k: v*9.80665 for k,v in accel.items()}
            gyro_dps = gyro.copy()

            accel_pitch = rad2deg(math.atan2(-accel_m['x'], math.sqrt(accel_m['y']**2 + accel_m['z']**2)))
            accel_roll = rad2deg(math.atan2(accel_m['y'], accel_m['z']))
            pitch = alpha_attitude*(pitch+gyro_dps['x']*dt)+(1-alpha_attitude)*accel_pitch
            roll = alpha_attitude*(roll+gyro_dps['y']*dt)+(1-alpha_attitude)*accel_roll
            yaw += gyro_dps['z']*dt

            a_nav = body_to_nav(accel_m, roll, pitch)
            a_nav['d'] -= 9.80665

            vel_n += a_nav['n']*dt
            vel_e += a_nav['e']*dt
            vel_n = kf_vn.update(vel_n)
            vel_e = kf_ve.update(vel_e)

            if not crashed:
                pos_n += vel_n*dt
                pos_e += vel_e*dt

            altitude = 0.0
            check_crash(accel_m, altitude, dt)

            # Mission state updates
            if mission_state=="BOOT": mission_state="ASCENT"
            elif mission_state=="ASCENT" and altitude>50: mission_state="APOGEE"
            elif mission_state=="APOGEE" and altitude<50: mission_state="DEPLOY"
            elif mission_state=="DEPLOY": mission_state="DESCENT"
            elif mission_state=="DESCENT": mission_state="NAVIGATION"
            elif mission_state=="NAVIGATION":
                dx = waypoint_n - pos_n
                dy = waypoint_e - pos_e
                if math.sqrt(dx*dx + dy*dy)<10: mission_state="WAYPOINT_REACHED"
            elif mission_state=="WAYPOINT_REACHED" and altitude<2: mission_state="LANDING"

            dx = waypoint_n - pos_n
            dy = waypoint_e - pos_e
            target_heading = rad2deg(math.atan2(dy, dx))
            yaw_error = target_heading - yaw
            while yaw_error>180: yaw_error-=360
            while yaw_error<-180: yaw_error+=360
            yaw += pid_yaw.compute(yaw_error, dt)*dt

            lat = start_lat + pos_n/m_per_deg_lat
            lon = start_lon + pos_e/m_per_deg_lon
            ts = time.time()
            packet = build_telemetry_packet(ts, altitude, vel_n, vel_e, lat, lon, pitch, yaw, mission_state, crashed)
            print(packet)

            elapsed = time.ticks_diff(time.ticks_ms(), now)/1000.0
            sleep_time = dt_nominal - elapsed
            if sleep_time>0: time.sleep(sleep_time)

            if mission_state=="LANDING":
                print("Mission complete: landing state reached.")
                break

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")
    except Exception as e:
        print("Fatal error:", e)

if __name__=="__main__":
    main_loop()
