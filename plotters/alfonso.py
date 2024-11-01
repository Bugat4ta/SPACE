import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def coes_to_state(coes):
    # Convert Classical Orbital Elements (COEs) to a Cartesian state vector
    a, e, i, RAAN, arg_peri, true_anomaly = coes
    i, RAAN, arg_peri, true_anomaly = np.radians([i, RAAN, arg_peri, true_anomaly])

    # Calculate distance and velocity in the perifocal coordinate system
    r = a * (1 - e ** 2) / (1 + e * np.cos(true_anomaly))
    x_p = r * np.cos(true_anomaly)
    y_p = r * np.sin(true_anomaly)
    z_p = 0
    v_x_p = -np.sqrt(398600 / a) * np.sin(true_anomaly)
    v_y_p = np.sqrt(398600 / a) * (e + np.cos(true_anomaly))
    v_z_p = 0

    # Rotation matrix from perifocal to ECI coordinates
    R = np.array([
        [np.cos(RAAN) * np.cos(arg_peri) - np.sin(RAAN) * np.sin(arg_peri) * np.cos(i),
         -np.cos(RAAN) * np.sin(arg_peri) - np.sin(RAAN) * np.cos(arg_peri) * np.cos(i),
         np.sin(RAAN) * np.sin(i)],
        [np.sin(RAAN) * np.cos(arg_peri) + np.cos(RAAN) * np.sin(arg_peri) * np.cos(i),
         -np.sin(RAAN) * np.sin(arg_peri) + np.cos(RAAN) * np.cos(arg_peri) * np.cos(i),
         -np.cos(RAAN) * np.sin(i)],
        [np.sin(arg_peri) * np.sin(i),
         np.cos(arg_peri) * np.sin(i),
         np.cos(i)]
    ])

    # Convert position and velocity to ECI frame
    r_eci = np.dot(R, np.array([x_p, y_p, z_p]))
    v_eci = np.dot(R, np.array([v_x_p, v_y_p, v_z_p]))

    return np.hstack((r_eci, v_eci))

def two_body_propagation(state, dt, mu=398600):
    # Apply simple 2-body propagation using Euler's method for demonstration
    r = state[:3]
    v = state[3:]
    r_mag = np.linalg.norm(r)
    a = -mu * r / r_mag ** 3
    state[:3] += v * dt
    state[3:] += a * dt
    return state

def propagate_orbit(initial_state, tspan, dt):
    # Propagate orbit using simple numerical integration (Euler's method)
    num_steps = int(tspan / dt)
    trajectory = np.zeros((num_steps, 6))
    state = initial_state.copy()

    for i in range(num_steps):
        trajectory[i] = state
        state = two_body_propagation(state, dt)
    
    return trajectory

def plot_orbits(states_list):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = ['crimson', 'lime', 'cyan', 'magenta']
    
    # Plot each orbit
    for i, states in enumerate(states_list):
        ax.plot(states[:, 0], states[:, 1], states[:, 2], color=colors[i], label=f'Orbit {i+1}')
    
    # Add the Earth at the center as a sphere
    earth_radius = 6371  # Earth's radius in km
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 50)
    x = earth_radius * np.outer(np.cos(u), np.sin(v))
    y = earth_radius * np.outer(np.sin(u), np.sin(v))
    z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    
    # Plot the Earth
    ax.plot_surface(x, y, z, color='blue', alpha=0.5, rstride=4, cstride=4, edgecolor='k')

    # Set limits for the axes
    max_radius = np.max(states_list) + 2000  # Add some margin
    ax.set_xlim([-max_radius, max_radius])
    ax.set_ylim([-max_radius, max_radius])
    ax.set_zlim([-max_radius, max_radius])

    # Labels and legend
    ax.set_xlabel("X (km)")
    ax.set_ylabel("Y (km)")
    ax.set_zlabel("Z (km)")
    ax.legend()
    plt.title('Satellite Orbits Around Earth')
    plt.show()

# Initial parameters
coes_list = [
    [15000.0, 0.4, 30, 0, 0, 30],
    [15000.0, 0.4, 30, 0, 40, 60],
    [15000.0, 0.4, 75, 0, 270, 60],
    [15000.0, 0.4, 100, 0, 270, 100]
]

tspan = 24 * 3600.0  # 1 day
dt = 10.0  # 10 seconds
states_list = []

# Calculate and propagate each orbit
for coes in coes_list:
    initial_state = coes_to_state(coes)
    trajectory = propagate_orbit(initial_state, tspan, dt)
    states_list.append(trajectory)

# Plot orbits
plot_orbits(states_list)

