import time
import math
from mpu6050 import mpu6050
from bmp388 import BMP388
import csv

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation = 0.0
        self.error_estimation = 1.0
        self.gain = 0.0
    def update(self, measurement):
        self.error_estimation += self.process_variance
        self.gain = self.error_estimation / (self.error_estimation + self.measurement_variance)
        self.estimation += self.gain * (measurement - self.estimation)
        self.error_estimation *= (1 - self.gain)
        return self.estimation

class PID:
    def __init__(self, kp, ki, kd, output_limits=(-10, 10)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.output_limits = output_limits
    def compute(self, error, dt):
        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral
        d = self.kd * (error - self.last_error) / dt
        self.last_error = error
        lo, hi = self.output_limits
        return max(min(p + i + d, hi), lo)

mpu = mpu6050(0x68)
bmp = BMP388()
alpha = 0.98
dt = 0.01
angle_x = 0
angle_y = 0
starting_latitude = 34.0000
starting_longitude = -117.0000
position_x = starting_latitude
position_y = starting_longitude
waypoint = (starting_latitude + 0.001, starting_longitude + 0.001)
velocity_x = 0
velocity_y = 0
kf_velocity_x = KalmanFilter(0.1, 1.0)
kf_velocity_y = KalmanFilter(0.1, 1.0)
pid_yaw = PID(kp=2.0, ki=0.1, kd=0.7, output_limits=(-5,5))
last_altitude = None
altitude_stable_time = 0
CRASH_ACCEL_THRESHOLD = 35
CRASH_ALT_TIMEOUT = 3
packet_counter = 0
mission_state = "BOOT"
crashed = False
log_file = "navigation_log.csv"
with open(log_file,"w",newline='') as f:
    csv.writer(f).writerow(["Time","Speed_X","Speed_Y","Pitch","Yaw","Altitude","Latitude","Longitude","MissionState","Crashed"])

def build_telemetry_packet(ts, alt, vx, vy, lat, lon, pitch, yaw, state, crash, batt=3.9):
    global packet_counter
    packet_counter += 1
    return f"$CANSAT,{packet_counter},{ts:.2f},{lat:.6f},{lon:.6f},{alt:.2f},{pitch:.2f},{yaw:.2f},{vx:.3f},{vy:.3f},{batt:.2f},{state},{int(crash)}*"

def log_data(t,vx,vy,pitch,yaw,alt,lat,lon,state,crash):
    with open(log_file,"a",newline='') as f:
        csv.writer(f).writerow([t,vx,vy,pitch,yaw,alt,lat,lon,state,crash])

def check_crash(accel,altitude):
    global last_altitude, altitude_stable_time, crashed, mission_state
    mag = math.sqrt(accel['x']**2 + accel['y']**2 + accel['z']**2)
    if mag > CRASH_ACCEL_THRESHOLD:
        crashed = True
        mission_state = "CRASHED"
        return True
    if last_altitude is None:
        last_altitude = altitude
    if abs(altitude - last_altitude) < 0.05:
        altitude_stable_time += dt
    else:
        altitude_stable_time = 0
        last_altitude = altitude
    if altitude_stable_time > CRASH_ALT_TIMEOUT:
        crashed = True
        mission_state = "CRASHED"
        return True
    return False

def navigate_to_waypoint():
    global position_x, position_y, angle_y, mission_state
    if crashed: return
    dist = math.sqrt((position_x - waypoint[0])**2 + (position_y - waypoint[1])**2)
    if dist < 0.0001:
        mission_state = "WAYPOINT_REACHED"
        return
    target_heading = math.degrees(math.atan2(waypoint[1]-position_y, waypoint[0]-position_x))
    error = target_heading - angle_y
    if error > 180: error -= 360
    if error < -180: error += 360
    angle_y += pid_yaw.compute(error, dt)

try:
    while True:
        try:
            accel = mpu.get_accel_data()
            gyro = mpu.get_gyro_data()
            altitude = bmp.read_altitude()
        except:
            continue

        if mission_state == "BOOT":
            mission_state = "ASCENT"

        if mission_state == "ASCENT":
            if altitude > 50:
                mission_state = "APOGEE"

        if mission_state == "APOGEE":
            if altitude < 50:
                mission_state = "DEPLOY"

        if mission_state == "DEPLOY":
            mission_state = "DESCENT"

        if mission_state == "DESCENT":
            navigate_to_waypoint()
            if abs(position_x - waypoint[0]) < 0.0001 and abs(position_y - waypoint[1]) < 0.0001:
                mission_state = "NAVIGATION"

        if mission_state == "NAVIGATION":
            navigate_to_waypoint()
            dist = math.sqrt((position_x - waypoint[0])**2 + (position_y - waypoint[1])**2)
            if dist < 0.0001:
                mission_state = "WAYPOINT_REACHED"

        if mission_state == "WAYPOINT_REACHED":
            if altitude < 2:
                mission_state = "LANDING"

        if not crashed:
            check_crash(accel, altitude)

        if crashed:
            velocity_x = 0
            velocity_y = 0

        gyro_x = gyro['x']/131
        gyro_y = gyro['y']/131
        accel_angle_x = math.degrees(math.atan2(accel['y'], accel['z']))
        accel_angle_y = math.degrees(math.atan2(accel['x'], accel['z']))
        angle_x = alpha*(angle_x + gyro_x*dt) + (1-alpha)*accel_angle_x
        angle_y = alpha*(angle_y + gyro_y*dt) + (1-alpha)*accel_angle_y
        filtered_vx = kf_velocity_x.update(accel['x'])
        filtered_vy = kf_velocity_y.update(accel['y'])
        if not crashed:
            position_x += filtered_vx*dt
            position_y += filtered_vy*dt
        t = time.time()
        packet = build_telemetry_packet(t, altitude, filtered_vx, filtered_vy, position_x, position_y, angle_x, angle_y, mission_state, crashed)
        print(packet)
        log_data(t, filtered_vx, filtered_vy, angle_x, angle_y, altitude, position_x, position_y, mission_state, crashed)
        if mission_state == "MISSION_COMPLETE": break
        time.sleep(dt)
except KeyboardInterrupt:
    pass
