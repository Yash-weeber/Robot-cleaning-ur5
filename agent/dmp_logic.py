import numpy as np
import math

try:
    from pydmps.dmp_discrete import DMPs_discrete
    from pydmps.dmp_rhythmic import DMPs_rhythmic
    MOVEMENT_PRIMITIVES_AVAILABLE = True
except ImportError:
    print("movement_primitives not found. Using custom implementation.")
    MOVEMENT_PRIMITIVES_AVAILABLE = False

class SimpleDMP:
    def __init__(self, n_dmps, n_bfs=50, dt=0.01, y0=None, goal=None):
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt

        # DMP parameters
        self.alpha_y = 25.0
        self.beta_y = self.alpha_y / 4.0
        self.alpha_x = 1.0

        # Gaussian basis functions
        self.centers = np.exp(-self.alpha_x * np.linspace(0, 1, n_bfs))
        self.widths = np.ones(n_bfs) * n_bfs / (self.centers[1:] - self.centers[:-1]).mean()

        # Weights for each DMP and basis function
        self.weights = np.zeros((n_dmps, n_bfs))

        # State variables
        self.reset_state()

        if y0 is not None:
            self.y0 = np.array(y0)
        if goal is not None:
            self.goal = np.array(goal)

    def reset_state(self):
        self.x = 1.0
        self.y = np.zeros(self.n_dmps)
        self.dy = np.zeros(self.n_dmps)

        if hasattr(self, 'y0'):
            self.y = self.y0.copy()
        if hasattr(self, 'goal'):
            self.goal_current = self.goal.copy()

    def step(self, tau=1.0, external_force=None):
        # Canonical system
        dx = -self.alpha_x * self.x * tau

        # Forcing function
        psi = np.exp(-self.widths * (self.x - self.centers) ** 2)
        psi_norm = psi / (psi.sum() + 1e-10)

        f = np.dot(self.weights, psi_norm) * self.x * (self.goal_current - self.y0)

        # Transformation system
        ddy = self.alpha_y * (self.beta_y * (self.goal_current - self.y) - self.dy / tau) + f
        if external_force is not None:
            ddy += external_force

        # Integration
        self.x += dx * self.dt
        self.dy += ddy * tau * self.dt
        self.y += self.dy * self.dt

        return self.y.copy()

    def imitate_path(self, path):
        # Learn from demonstration
        path = np.array(path)
        if len(path.shape) == 1:
            path = path.reshape(-1, 1)

        n_points, n_dmps = path.shape
        self.n_dmps = n_dmps
        self.weights = np.zeros((n_dmps, self.n_bfs))

        self.y0 = path[0]
        self.goal = path[-1]

        # Generate target forcing function
        dt = 1.0 / n_points
        x_track = np.exp(-self.alpha_x * np.linspace(0, 1, n_points))

        # Calculate target accelerations
        velocity = np.gradient(path, axis=0) / dt
        acceleration = np.gradient(velocity, axis=0) / dt

        for d in range(n_dmps):
            f_target = acceleration[:, d] - self.alpha_y * (
                    self.beta_y * (self.goal[d] - path[:, d]) - velocity[:, d]
            )

            # Regression to find weights
            X = np.zeros((n_points, self.n_bfs))
            for i, x in enumerate(x_track):
                psi = np.exp(-self.widths * (x - self.centers) ** 2)
                X[i] = psi * x * (self.goal[d] - self.y0[d])

            self.weights[d] = np.linalg.pinv(X) @ f_target

class SimpleRythmicDMP:
    def __init__(self, n_dmps, n_bfs=50, dt=0.01):
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt

        # Rhythmic DMP parameters
        self.alpha_y = 25.0
        self.beta_y = self.alpha_y / 4.0

        # Phase and frequency
        self.phi = 0.0
        self.freq = 1.0  # Hz

        # Basis functions for rhythmic patterns
        self.centers = np.linspace(0, 2 * np.pi, n_bfs)
        self.widths = np.ones(n_bfs) * n_bfs / (2 * np.pi)

        # Weights and amplitudes
        self.weights = np.zeros((n_dmps, n_bfs))
        self.r = np.ones(n_dmps)  # amplitude

        # State variables
        self.reset_state()

    def reset_state(self):
        self.phi = 0.0
        self.y = np.zeros(self.n_dmps)
        self.dy = np.zeros(self.n_dmps)
        self.r = np.ones(self.n_dmps)

    def step(self, tau=1.0):
        # Phase system
        dphi = 2 * np.pi * self.freq * tau

        # Forcing function
        psi = np.exp(-self.widths * np.cos(self.phi - self.centers))
        psi_norm = psi / (psi.sum() + 1e-10)

        f = np.dot(self.weights, psi_norm) * self.r

        # Rhythmic transformation system
        ddy = self.alpha_y * (self.beta_y * (-self.y) - self.dy / tau) + f

        # Integration
        self.phi += dphi * self.dt
        if self.phi > 2 * np.pi:
            self.phi -= 2 * np.pi

        self.dy += ddy * tau * self.dt
        self.y += self.dy * self.dt

        return self.y.copy()

    def imitate_path(self, path):
        # Learn rhythmic pattern from demonstration
        path = np.array(path)
        if len(path.shape) == 1:
            path = path.reshape(-1, 1)

        n_points, n_dmps = path.shape
        self.n_dmps = n_dmps
        self.weights = np.zeros((n_dmps, self.n_bfs))

        # Calculate amplitude
        self.r = np.std(path, axis=0)

        # Generate phase trajectory
        phi_track = np.linspace(0, 2 * np.pi, n_points)

        # Calculate target accelerations
        dt = 1.0 / n_points
        velocity = np.gradient(path, axis=0) / dt
        acceleration = np.gradient(velocity, axis=0) / dt

        for d in range(n_dmps):
            f_target = acceleration[:, d] - self.alpha_y * (
                    self.beta_y * (-path[:, d]) - velocity[:, d]
            )

            # Regression to find weights
            X = np.zeros((n_points, self.n_bfs))
            for i, phi in enumerate(phi_track):
                psi = np.exp(-self.widths * np.cos(phi - self.centers))
                X[i] = psi * self.r[d]

            self.weights[d] = np.linalg.pinv(X) @ f_target