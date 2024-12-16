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

# Simulator parameters
SIMULATION_TIME = 100  # total time for simulation in seconds
DT = 0.01  # time step in seconds
NUM_SAMPLES = int(SIMULATION_TIME / DT)

# Initialize Kalman Filters
kf_velocity_x = KalmanFilter(process_variance=0.1, measurement_variance=1.0)
kf_velocity_y = KalmanFilter(process_variance=0.1, measurement_variance=1.0)

# Initial positions, angles, and velocities
angle_x, angle_y = 0, 0
velocity_x, velocity_y = 0, 0
position_x, position_y = 0, 0
altitude = 1  # Initial altitude in meters

# Waypoint definition with altitude
waypoint_x = 700
waypoint_y = 300
waypoint_altitude = 150  # Target altitude for the waypoint (in meters)
waypoint_reached = False  # Flag to check if the waypoint is reached

# Abort mode parameters
abort_distance = 50  # Distance threshold (in meters) for triggering abort mode
abort_mode = False  # Flag to indicate whether abort mode is active

log_file = 'navigation_log.csv'

def log_data(time_stamp, speed_x, speed_y, angle_x, angle_y, altitude):
    with open(log_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time_stamp, speed_x, speed_y, angle_x, angle_y, altitude])

# Create and write header to CSV file
with open(log_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Speed_X', 'Speed_Y', 'Angle_X', 'Angle_Y', 'Altitude'])

try:
    for sample in range(NUM_SAMPLES):
        current_time = sample * DT  # Use simulation time instead of system time

        # Simulated sensor data generation
        accel_data_x = random.uniform(-2, 2)  # Simulated accelerometer data in m/s^2
        accel_data_y = random.uniform(-2, 2)
        gyro_data_x = random.uniform(-180, 180)  # Simulated gyroscope data in degrees/s
        gyro_data_y = random.uniform(-180, 180)
        altitude += random.uniform(0.1, 0.5)  # Gradually increase altitude

        gyro_x = gyro_data_x / 131
        gyro_y = gyro_data_y / 131

        accel_angle_x = math.degrees(math.atan2(accel_data_y, 1))
        accel_angle_y = math.degrees(math.atan2(accel_data_x, 1))

        angle_x = 0.98 * (angle_x + gyro_x * DT) + 0.02 * accel_angle_x
        angle_y = 0.98 * (angle_y + gyro_y * DT) + 0.02 * accel_angle_y

        filtered_velocity_x = kf_velocity_x.update(accel_data_x)
        filtered_velocity_y = kf_velocity_y.update(accel_data_y)

        # Calculate direction to the waypoint
        direction_x = waypoint_x - position_x
        direction_y = waypoint_y - position_y
        distance = math.sqrt(direction_x**2 + direction_y**2)

        # Normalize direction
        if distance > 0:
            normalized_direction_x = direction_x / distance
            normalized_direction_y = direction_y / distance
        else:
            normalized_direction_x = 0
            normalized_direction_y = 0

        # Check if the waypoint has been reached
        if distance < 1.0 and abs(altitude - waypoint_altitude) < 1.0 and not waypoint_reached:  # Threshold for reaching the waypoint
            waypoint_reached = True
            print(f"Waypoint reached at position ({position_x:.2f}, {position_y:.2f}, {altitude:.2f})")

        # Check if we're within the abort distance from the waypoint
        if distance < abort_distance and not abort_mode:
            abort_mode = True
            print("Abort euler, Point reached.")

        # Abort Mode - Ascend vertically and stop horizontal movement
        if abort_mode:
            # Point upward by adjusting the pitch (set angle_x to a high positive value)
            angle_x = 45  # Assume pointing upward
            altitude += 0.5  # Increase altitude in abort mode

            # Stop horizontal velocity (no movement towards waypoint)
            velocity_x = 0
            velocity_y = 0
        else:
            # Adjust velocity towards the waypoint
            speed_factor = 1.0
            if distance < 10:  # Slow down when close to the waypoint
                speed_factor = distance / 10

            velocity_x += normalized_direction_x * speed_factor
            velocity_y += normalized_direction_y * speed_factor

            # Update position towards the waypoint
            position_x += velocity_x * DT
            position_y += velocity_y * DT

        log_data(current_time, filtered_velocity_x, filtered_velocity_y, angle_x, angle_y, altitude)

        # Print status
        print(f"Position: ({position_x:.2f}, {position_y:.2f}, {altitude:.2f} m) | Target: ({waypoint_x}, {waypoint_y}, {waypoint_altitude}) | Altitude: {altitude:.2f} m")

        time.sleep(DT)

except KeyboardInterrupt:
    print("Simulation terminated.")
finally:
    pass
