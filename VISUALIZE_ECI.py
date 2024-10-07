import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_orbit_and_distance(position1, position2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    R_earth = 6371e3  # Earth's radius in meters
    x_earth = R_earth * np.outer(np.cos(u), np.sin(v))
    y_earth = R_earth * np.outer(np.sin(u), np.sin(v))
    z_earth = R_earth * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x_earth, y_earth, z_earth, color='b', alpha=0.5)

    ax.scatter(position1[0], position1[1], position1[2], color='r', label='Position 1 (t1)')
    ax.scatter(position2[0], position2[1], position2[2], color='g', label='Position 2 (t2)')

    ax.plot([position1[0], position2[0]], [position1[1], position2[1]], [position1[2], position2[2]], 'k--', label='Distance')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    ax.legend()

    plt.show()

inclination = 51.6451
raan = 279.0
eccentricity = 0.0004
arg_perigee = 50.2771
altitude = 400e3  
M0 = 0

t1 = 3600  
t2 = 7200  

M1 = calculate_mean_anomaly_at_time(M0, n, t1, t0)
M2 = calculate_mean_anomaly_at_time(M0, n, t2, t0)

position1 = compute_position(inclination, raan, eccentricity, arg_perigee, M1, altitude)
position2 = compute_position(inclination, raan, eccentricity, arg_perigee, M2, altitude)

plot_orbit_and_distance(position1, position2)
