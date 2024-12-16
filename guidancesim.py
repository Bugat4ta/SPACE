import time
import math
import random
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

SIMULATION_TIME = 100
DT = 0.01
NUM_SAMPLES = int(SIMULATION_TIME / DT)

kf_velocity_x = KalmanFilter(process_variance=0.1, measurement_variance=1.0)
kf_velocity_y = KalmanFilter(process_variance=0.1, measurement_variance=1.0)

angle_x, angle_y = 0, 0
velocity_x, velocity_y = 0, 0
position_x, position_y = 0, 0
altitude = 100

waypoint_x = 50
waypoint_y = 50
waypoint_altitude = 150
waypoint_reached = False

abort_distance = 5
abort_mode = False

log_file = 'navigation_log.csv'

def log_data(time_stamp, speed_x, speed_y, angle_x, angle_y, altitude):
    with open(log_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time_stamp, speed_x, speed_y, angle_x, angle_y, altitude])

with open(log_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Speed_X', 'Speed_Y', 'Angle_X', 'Angle_Y', 'Altitude'])

try:
    for sample in range(NUM_SAMPLES):
        current_time = sample * DT

        accel_data_x = random.uniform(-2, 2)
        accel_data_y = random.uniform(-2, 2)
        gyro_data_x = random.uniform(-180, 180)
        gyro_data_y = random.uniform(-180, 180)
        altitude += random.uniform(0.1, 0.5)

        gyro_x = gyro_data_x / 131
        gyro_y = gyro_data_y / 131

        accel_angle_x = math.degrees(math.atan2(accel_data_y, 1))
        accel_angle_y = math.degrees(math.atan2(accel_data_x, 1))

        angle_x = 0.98 * (angle_x + gyro_x * DT) + 0.02 * accel_angle_x
        angle_y = 0.98 * (angle_y + gyro_y * DT) + 0.02 * accel_angle_y

        filtered_velocity_x = kf_velocity_x.update(accel_data_x)
        filtered_velocity_y = kf_velocity_y.update(accel_data_y)

        direction_x = waypoint_x - position_x
        direction_y = waypoint_y - position_y
        distance = math.sqrt(direction_x**2 + direction_y**2)

        if distance > 0:
            normalized_direction_x = direction_x / distance
            normalized_direction_y = direction_y / distance
        else:
            normalized_direction_x = 0
            normalized_direction_y = 0

        if distance < 1.0 and abs(altitude - waypoint_altitude) < 1.0 and not waypoint_reached:
            waypoint_reached = True
            print(f"Waypoint reached at position ({position_x:.2f}, {position_y:.2f}, {altitude:.2f})")

        if distance < abort_distance and not abort_mode:
            abort_mode = True
            print("Abort mode activated! Pointing upwards.")

        if abort_mode:
            angle_x = 45
            altitude += 0.5
            velocity_x = 0
            velocity_y = 0
        else:
            speed_factor = 1.0
            if distance < 10:
                speed_factor = distance / 10

            velocity_x += normalized_direction_x * speed_factor
            velocity_y += normalized_direction_y * speed_factor

            position_x += velocity_x * DT
            position_y += velocity_y * DT

        log_data(current_time, filtered_velocity_x, filtered_velocity_y, angle_x, angle_y, altitude)

        print(f"Position: ({position_x:.2f}, {position_y:.2f}, {altitude:.2f} m) | Target: ({waypoint_x}, {waypoint_y}, {waypoint_altitude}) | Altitude: {altitude:.2f} m")

        time.sleep(DT)

except KeyboardInterrupt:
    print("Simulation terminated.")
finally:
    pass
