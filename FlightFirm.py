import utime as time
import math
import random

# ---------------- Helper Functions ----------------
def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def meters_per_deg_lat(lat_deg):
    return 111132.954 - 559.822*math.cos(2*deg2rad(lat_deg)) + 1.175*math.cos(4*deg2rad(lat_deg))

def meters_per_deg_lon(lat_deg):
    return 111412.84*math.cos(deg2rad(lat_deg)) - 93.5*math.cos(3*deg2rad(lat_deg))

def compute_checksum(payload):
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return "%02X" % c

# ---------------- Kalman & PID ----------------
class KalmanFilter:
    def __init__(self, q=0.1, r=1.0, x0=0.0, p0=1.0):
        self.q, self.r, self.x, self.p = q, r, x0, p0
    def update(self, z):
        self.p += self.q
        k = self.p/(self.p+self.r)
        self.x += k*(z-self.x)
        self.p *= (1-k)
        return self.x

class PID:
    def __init__(self, kp, ki, kd, output_limits=(-999,999)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.last_error = None
        self.output_limits = output_limits
    def compute(self, error, dt):
        if dt <= 0: return 0.0
        p = self.kp*error
        self.integral += error*dt
        i = self.ki*self.integral
        d = 0.0 if self.last_error is None else self.kd*((error-self.last_error)/dt)
        self.last_error = error
        out = p + i + d
        lo, hi = self.output_limits
        return max(min(out, hi), lo)

def body_to_nav(accel_body, roll_deg, pitch_deg):
    phi = deg2rad(roll_deg)
    theta = deg2rad(pitch_deg)
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
    payload = "CANSAT,%d,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.3f,%.3f,%.2f,%s,%d" % (
        packet_counter, ts, lat, lon, alt, pitch, yaw, vn, ve, batt, state, int(crash)
    )
    chk = compute_checksum(payload)
    return "$%s*%s" % (payload, chk)

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

# ---------------- Sensor Classes ----------------
# Dummy sensor for simulation
class DummyQMI8658:
    def get_accel_data(self):
        return {'x': random.uniform(-0.1,0.1),
                'y': random.uniform(-0.1,0.1),
                'z': random.uniform(0.9,1.1)}
    def get_gyro_data(self):
        return {'x': random.uniform(-1,1),
                'y': random.uniform(-1,1),
                'z': random.uniform(-5,5)}

# Real sensor
def get_real_sensor():
    try:
        from machine import I2C, Pin
        i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)
        class QMI8658:
            def __init__(self, i2c, addr=0x6B):
                self.i2c = i2c
                self.addr = addr
                # Reset and enable sensors
                self.write_reg(0x7E, 0xB6)
                time.sleep_ms(50)
                self.write_reg(0x7C, 0x01)
                self.write_reg(0x7D, 0x01)
            def write_reg(self, reg, val):
                self.i2c.writeto_mem(self.addr, reg, bytes([val]))
            def read_reg(self, reg, nbytes=1):
                buf = bytearray(nbytes)
                self.i2c.readfrom_mem_into(self.addr, reg, buf)
                return buf
            def get_accel_data(self):
                raw = self.read_reg(0x0D,6)
                x = int.from_bytes(raw[0:2],'little',signed=True)/1000.0
                y = int.from_bytes(raw[2:4],'little',signed=True)/1000.0
                z = int.from_bytes(raw[4:6],'little',signed=True)/1000.0
                return {'x':x,'y':y,'z':z}
            def get_gyro_data(self):
                raw = self.read_reg(0x12,6)
                x = int.from_bytes(raw[0:2],'little',signed=True)/16.4
                y = int.from_bytes(raw[2:4],'little',signed=True)/16.4
                z = int.from_bytes(raw[4:6],'little',signed=True)/16.4
                return {'x':x,'y':y,'z':z}
        return QMI8658(i2c)
    except Exception as e:
        print("Real sensor not found, using simulation:", e)
        return DummyQMI8658()

# ---------------- Initialization ----------------
mpu = get_real_sensor()
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
waypoint_n = (waypoint_lat - start_lat)*m_per_deg_lat
waypoint_e = (waypoint_lon - start_lon)*m_per_deg_lon
pid_yaw = PID(2.0,0.05,0.4,(-5,5))
packet_counter = 0

# ---------------- Main Loop ----------------
def main_loop():
    global pos_n, pos_e, vel_n, vel_e, pitch, roll, yaw, crashed, mission_state
    last_time = time.ticks_ms()
    try:
        while True:
            now = time.ticks_ms()
            dt = max(time.ticks_diff(now,last_time)/1000.0, dt_nominal)
            last_time = now

            accel = mpu.get_accel_data()
            gyro = mpu.get_gyro_data()

            accel_m = {k:v*9.80665 for k,v in accel.items()}
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

            altitude = 0.0  # replace with real barometer if available
            check_crash(accel_m, altitude, dt)

            # Mission state logic
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
            while yaw_error>180: yaw_error -= 360
            while yaw_error<-180: yaw_error += 360
            yaw += pid_yaw.compute(yaw_error, dt)*dt

            lat = start_lat + pos_n/m_per_deg_lat
            lon = start_lon + pos_e/m_per_deg_lon
            ts = time.time()
            packet = build_telemetry_packet(ts, altitude, vel_n, vel_e, lat, lon, pitch, yaw, mission_state, crashed)
            print(packet)

            time.sleep(dt_nominal)
            if mission_state=="LANDING":
                print("Mission complete: landing state reached.")
                break

    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")

if __name__=="__main__":
    main_loop()
