import numpy as np
import matplotlib.pyplot as plt

G = 6.67430e-11  # m^3 kg^-1 s^-2, gravitational constant
M_earth = 5.972e24  # kg, mass of Earth
R_earth = 6.371e6  # m, radius of Earth
satellite_altitude = 400000  # 400 km altitude
satellite_orbit_radius = R_earth + satellite_altitude  # meters

def orbital_velocity(mass, orbit_radius):
    return np.sqrt(G * mass / orbit_radius)

v_orbit = orbital_velocity(M_earth, satellite_orbit_radius)

# Orbital period (T): T = 2 * pi * r / v
T_orbit = 2 * np.pi * satellite_orbit_radius / v_orbit

def satellite_position_velocity(t, orbit_radius, period, v_orbit):
    theta = (2 * np.pi / period) * t  # radians
   
    x_pos = orbit_radius * np.cos(theta)
    y_pos = orbit_radius * np.sin(theta)

    x_vel = -v_orbit * np.sin(theta)
    y_vel = v_orbit * np.cos(theta)
    
    return (x_pos, y_pos), (x_vel, y_vel)

time = 2000  # seconds (you can change this to any value within the orbital period)

position, velocity = satellite_position_velocity(time, satellite_orbit_radius, T_orbit, v_orbit)

print(f"At time {time} seconds:")
print(f"Position (x, y): {position} meters")
print(f"Velocity (x, y): {velocity} m/s")

time_steps = 1000
theta_values = np.linspace(0, 2 * np.pi, time_steps)
x_orbit = satellite_orbit_radius * np.cos(theta_values)
y_orbit = satellite_orbit_radius * np.sin(theta_values)

plt.plot(x_orbit, y_orbit, label='Satellite Orbit')
plt.plot(position[0], position[1], 'ro', label='Satellite Position')  # Current satellite position
plt.gca().set_aspect('equal', adjustable='box')
plt.title(f"Satellite Orbit and Position at t = {time} seconds")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid(True)
plt.legend()
plt.show()
