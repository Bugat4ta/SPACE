import numpy as np

def geo_to_eci(latitude, longitude):
    """Convert geographic coordinates to ECI coordinates."""
    lat_rad = np.radians(latitude)
    lon_rad = np.radians(longitude)
    R = 6371e3  # Earth's radius in meters
    x = R * np.cos(lat_rad) * np.cos(lon_rad)
    y = R * np.cos(lat_rad) * np.sin(lon_rad)
    z = R * np.sin(lat_rad)
    return np.array([x, y, z])

def compute_position(inclination, raan, eccentricity, arg_perigee, mean_anomaly, altitude):
    """Compute the ECI position of a satellite based on its orbital elements."""
    G = 6.67430e-11  # Gravitational constant
    M_earth = 5.972e24  # Earth's mass in kg
    inclination = np.radians(inclination)
    raan = np.radians(raan)
    arg_perigee = np.radians(arg_perigee)
    
    R_earth = 6371e3  # Earth's radius in meters
    a = R_earth + altitude  # Semi-major axis is Earth's radius + altitude
    
    M = np.radians(mean_anomaly)
    r = a * (1 - eccentricity ** 2) / (1 + eccentricity * np.cos(M))  # Distance from Earth's center
    
    # Calculate position in orbital plane (assuming circular orbit for simplicity)
    x_orbit = r * np.cos(M)
    y_orbit = r * np.sin(M)
    
    # Convert to ECI coordinates (simplified)
    position_eci = np.array([
        x_orbit * (np.cos(raan) * np.cos(arg_perigee) - np.sin(raan) * np.sin(arg_perigee) * np.cos(inclination)),
        y_orbit * (np.sin(raan) * np.cos(arg_perigee) + np.cos(raan) * np.sin(arg_perigee) * np.cos(inclination)),
        y_orbit * (np.sin(arg_perigee) * np.sin(inclination))
    ])
    
    return position_eci

def calculate_mean_anomaly_at_time(M0, n, t, t0):
    """Calculate mean anomaly at a given time."""
    return M0 + n * (t - t0)

def calculate_distance_and_time(inclination, raan, eccentricity, arg_perigee, altitude, M1, M2):
    """Calculate distance and time difference between two satellite positions."""
    position1 = compute_position(inclination, raan, eccentricity, arg_perigee, M1, altitude)
    position2 = compute_position(inclination, raan, eccentricity, arg_perigee, M2, altitude)
    
    # Calculate distance between two ECI positions
    distance = np.linalg.norm(position2 - position1)
    
    # Calculate mean motion (assuming 90-minute orbit period)
    T = 5400  # seconds (90 minutes)
    n = 2 * np.pi / T  # Mean motion (rad/s)
    
    # Calculate time difference using mean anomaly
    M1_rad = np.radians(M1)
    M2_rad = np.radians(M2)
    
    time_difference = (M2_rad - M1_rad) / n
    if time_difference < 0:
        time_difference += (2 * np.pi) / n
    
    return distance, time_difference

# Example usage

# Define two times for which we want to calculate positions (in seconds)
t0 = 0  # reference time (start)
t1 = 3600  # 1 hour later
t2 = 7200  # 2 hours later

# Orbital parameters
inclination = 51.6451
raan = 279.0
eccentricity = 0.0004
arg_perigee = 50.2771
M0 = 0  # Mean anomaly at t0 (starting point)
altitude = 400e3  

T = 5580  # seconds (90 minutes)
n = 2 * np.pi / T  # Mean motion (rad/s)

# Calculate mean anomalies at the two times
M1 = calculate_mean_anomaly_at_time(M0, n, t1, t0)
M2 = calculate_mean_anomaly_at_time(M0, n, t2, t0)

# Calculate the distance and time difference between two positions
distance, time_difference = calculate_distance_and_time(inclination, raan, eccentricity, arg_perigee, altitude, M1, M2)

print(f"Mean Anomaly at t1 (M1): {M1:.2f} degrees")
print(f"Mean Anomaly at t2 (M2): {M2:.2f} degrees")
print(f"Distance between points: {distance:.2f} meters")
print(f"Time difference between points: {time_difference:.2f} seconds")
