import numpy as np
import matplotlib.pyplot as plt
plt.style.use('dark_background')

earth_radius = 6378.0
earth_mu = 3.9860043543609598E+05

def two_body_ode(t, state):
    r = state[:3]
    a = -earth_mu * r / np.linalg.norm(r) ** 3
    return np.array([state[3], state[4], state[5], a[0], a[1], a[2]])

def rk4_step(f, t, y, h):
    k1 = f(t, y)
    k2 = f(t + 0.5 * h, y + 0.5 * k1 * h)
    k3 = f(t + 0.5 * h, y + 0.5 * k2 * h)
    k4 = f(t + h, y + k3 * h)
    return y + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

def plot_orbits(rs, args):
    _args = {
        'figsize': (10, 8),
        'labels': ['Orbit 1'],
        'colors': ['m'],
        'traj_lws': 3,
        'dist_unit': 'km',
        'cb_radius': 6378.0,
        'cb_cmap': 'Blues',
        'legend': True,
        'show': True,
    }

    fig = plt.figure(figsize=_args['figsize'])
    ax = fig.add_subplot(111, projection='3d')

    for r in rs:
        ax.plot(r[:, 0], r[:, 1], r[:, 2], color=_args['colors'][0], label=_args['labels'][0], zorder=10, linewidth=_args['traj_lws'])

    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    x = _args['cb_radius'] * np.cos(u) * np.sin(v)
    y = _args['cb_radius'] * np.sin(u) * np.sin(v)
    z = _args['cb_radius'] * np.cos(v)
    ax.plot_surface(x, y, z, cmap=_args['cb_cmap'], zorder=1)

    ax.set_xlabel('X (km)')
    ax.set_ylabel('Y (km)')
    ax.set_zlabel('Z (km)')
    ax.legend()
    
    if _args['show']:
        plt.show()

    plt.close()

if __name__ == '__main__':
    altitude = float(input("Enter the altitude of the orbit (km): "))
    inclination = float(input("Enter the inclination of the orbit (degrees): "))
    eccentricity = float(input("Enter the orbital eccentricity (0 for circular, 0 to 1 for elliptical): "))

    r0_norm = earth_radius + altitude
    v0_norm = (earth_mu / r0_norm) ** 0.5

    if eccentricity != 0:
        v0_norm *= (1 + eccentricity)

    inclination_rad = np.radians(inclination)
    statei = [
        r0_norm * np.cos(inclination_rad),
        r0_norm * np.sin(inclination_rad),
        0,
        0,
        v0_norm * np.cos(inclination_rad),
        v0_norm * np.sin(inclination_rad)
    ]

    tspan = 100.0 * 60.0
    dt = 100.0
    steps = int(tspan / dt)
    ets = np.zeros((steps, 1))
    states = np.zeros((steps, 6))
    states[0] = statei

    for step in range(steps - 1):
        states[step + 1] = rk4_step(two_body_ode, ets[step], states[step], dt)

    plot_orbits([states], {'show': True})
