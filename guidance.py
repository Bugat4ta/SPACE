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

mpu = mpu6050(0x68)
bmp = BMP388()

alpha = 0.98
dt = 0.01
angle_x, angle_y = 0, 0

starting_latitude = 34.0000
starting_longitude = -117.0000

kf_velocity_x = KalmanFilter(process_variance=0.1, measurement_variance=1.0)
kf_velocity_y = KalmanFilter(process_variance=0.1, measurement_variance=1.0)

velocity_x, velocity_y = 0, 0
position_x, position_y = 0, 0

log_file = 'navigation_log.csv'

with open(log_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Speed_X', 'Speed_Y', 'Angle_X', 'Angle_Y', 'Altitude', 'Starting_Latitude', 'Starting_Longitude'])

def log_data(time_stamp, speed_x, speed_y, angle_x, angle_y, altitude):
    with open(log_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time_stamp, speed_x, speed_y, angle_x, angle_y, altitude, starting_latitude, starting_longitude])

try:
    while True:
        try:
            accel_data = mpu.get_accel_data()
        except Exception as e:
            print(f"Error reading acceleration data: {e}")
            continue

        try:
            gyro_data = mpu.get_gyro_data()
        except Exception as e:
            print(f"Error reading gyroscope data: {e}")
            continue

        try:
            altitude = bmp.read_altitude()
        except Exception as e:
            print(f"Error reading altitude data: {e}")
            altitude = None

        gyro_x = gyro_data['x'] / 131
        gyro_y = gyro_data['y'] / 131

        accel_angle_x = math.degrees(math.atan2(accel_data['y'], accel_data['z']))
        accel_angle_y = math.degrees(math.atan2(accel_data['x'], accel_data['z']))

        angle_x = alpha * (angle_x + gyro_x * dt) + (1 - alpha) * accel_angle_x
        angle_y = alpha * (angle_y + gyro_y * dt) + (1 - alpha) * accel_angle_y

        filtered_velocity_x = kf_velocity_x.update(accel_data['x'])
        filtered_velocity_y = kf_velocity_y.update(accel_data['y'])

        position_x += filtered_velocity_x * dt
        position_y += filtered_velocity_y * dt

        current_time = time.time()

        log_data(current_time, filtered_velocity_x, filtered_velocity_y, angle_x, angle_y, altitude)

        print(f"Pitch: {angle_x:.2f}° | Roll: {angle_y:.2f}° | Altitude: {altitude if altitude is not None else 'N/A'} m | Starting Location: ({starting_latitude}, {starting_longitude})")

        time.sleep(dt)

except KeyboardInterrupt:
    print("Guidance system terminated.")

finally:
    pass
