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

# Sensor initialization
mpu = mpu6050(0x68)
bmp = BMP388()

# Complementary filter parameters
alpha = 0.98
dt = 0.01
angle_x, angle_y = 0, 0

# Starting position, waypoint, and threshold
starting_latitude = 34.0000
starting_longitude = -117.0000
waypoint = (starting_latitude + 0.001, starting_longitude + 0.001)  # Example waypoint
waypoint_threshold = 0.0001  # Threshold distance in degrees
waypoint_reached = False

# Kalman filters for x and y velocities
kf_velocity_x = KalmanFilter(process_variance=0.1, measurement_variance=1.0)
kf_velocity_y = KalmanFilter(process_variance=0.1, measurement_variance=1.0)

# Initial velocities and positions
velocity_x, velocity_y = 0, 0
position_x, position_y = starting_latitude, starting_longitude

# Log file setup
log_file = 'navigation_log.csv'
with open(log_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time', 'Speed_X', 'Speed_Y', 'Angle_X', 'Angle_Y', 'Altitude', 'Latitude', 'Longitude'])

def log_data(time_stamp, speed_x, speed_y, angle_x, angle_y, altitude, latitude, longitude):
    with open(log_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([time_stamp, speed_x, speed_y, angle_x, angle_y, altitude, latitude, longitude])

def navigate_to_waypoint():
    global waypoint_reached, position_x, position_y, angle_x, angle_y
    # Calculate distance to the waypoint
    distance_to_waypoint = math.sqrt((position_x - waypoint[0])**2 + (position_y - waypoint[1])**2)
    if distance_to_waypoint < waypoint_threshold:  # Close enough to waypoint
        waypoint_reached = True
    else:
        angle_to_waypoint = math.degrees(math.atan2(waypoint[1] - position_y, waypoint[0] - position_x))
        angle_diff = angle_to_waypoint - angle_y
        
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        # Simple steering logic
        if angle_diff > 0:
            angle_y += 0.5  # Turn right
        else:
            angle_y -= 0.5  # Turn left

def update_altitude():
    global angle_x
    if angle_x < 0:  # Negative pitch decreases altitude
        return -0.5
    elif angle_x > 20:  # High pitch slows altitude increase
        return 0.5
    else:  # Maintain altitude or slight increase
        return 1

try:
    while True:
        try:
            accel_data = mpu.get_accel_data()
            gyro_data = mpu.get_gyro_data()
            altitude = bmp.read_altitude()
        except Exception as e:
            print(f"Error reading sensor data: {e}")
            continue

        gyro_x = gyro_data['x'] / 131
        gyro_y = gyro_data['y'] / 131

        accel_angle_x = math.degrees(math.atan2(accel_data['y'], accel_data['z']))
        accel_angle_y = math.degrees(math.atan2(accel_data['x'], accel_data['z']))

        angle_x = alpha * (angle_x + gyro_x * dt) + (1 - alpha) * accel_angle_x
        angle_y = alpha * (angle_y + gyro_y * dt) + (1 - alpha) * accel_angle_y

        filtered_velocity_x = kf_velocity_x.update(accel_data['x'])
        filtered_velocity_y = kf_velocity_y.update(accel_data['y'])

        # Update positions
        position_x += filtered_velocity_x * dt
        position_y += filtered_velocity_y * dt

        # Navigate to waypoint
        navigate_to_waypoint()

        # Adjust altitude
        altitude_change = update_altitude()
        altitude += altitude_change

        current_time = time.time()
        log_data(current_time, filtered_velocity_x, filtered_velocity_y, angle_x, angle_y, altitude, position_x, position_y)

        print(f"Pitch: {angle_x:.2f}° | Roll: {angle_y:.2f}° | Altitude: {altitude:.2f} m | Position: ({position_x:.6f}, {position_y:.6f}) | Waypoint Reached: {waypoint_reached}")

        if waypoint_reached:
            print("Waypoint has been reached.")
            break

        time.sleep(dt)

except KeyboardInterrupt:
    print("Guidance system terminated.")

finally:
    pass
