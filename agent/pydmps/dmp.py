import numpy as np
from .CanonicalSystem import CanonicalSystem

class DMPs:
    def __init__(self, n_dmps, n_bfs, dt=0.01, y0=0, goal=1, w=None, ay=None, by=None, **kwargs):
        '''
        Initializes the DMPs system.
        Parameters:
        - n_dmps: Number of dynamic movement primitives (DMPs).
        - n_bfs: Number of basis functions per DMP.
        - dt: Time step for integration.
        - y0: Initial position.
        - goal: Goal position.
        - w: Weights for the basis functions. If None, initialized to zeros.
        - ay int: gain on attractor term y dynamics
        - by int: gain on attractor term y dynamics
        '''
        self.n_dmps = n_dmps  # Number of DMPs
        self.n_bfs = n_bfs    # Number of basis functions per DMP
        self.dt = dt          # Time step
        self.y0 = np.ones(n_dmps) * y0  # Initial position
        self.goal = np.ones(n_dmps) * goal  # Goal position
        if w is None:
            self.w = np.zeros((n_dmps, n_bfs))  # Weights for basis functions
        else:
            self.w = w
        ay = ay if ay is not None else 25.0
        by = by if by is not None else ay / 4.0
        self.ay = np.ones(n_dmps) * ay      # Gain term for attractor dynamics
        self.by = np.ones(n_dmps) * by      # Gain term for attractor dynamics

        # Initialize canonical system
        self.cs = CanonicalSystem(dt=self.dt, **kwargs)

        self.timesteps = self.cs.timesteps  # Number of time steps
        
        # State variables
        self.reset_state()

    def reset_state(self, y0=None, dy=None, ddy=None):
        """Reset the system state"""
        if y0 is not None:
            self.y0 = y0.copy()
        self.y = self.y0.copy()
        if dy is not None:
            self.dy = dy.copy()
        else:
            self.dy = np.zeros(self.n_dmps)
        if ddy is not None:
            self.ddy = ddy.copy()
        else:
            self.ddy = np.zeros(self.n_dmps)
        self.cs.reset()

    def check_offset(self):
        '''Check if the goal is too close to the start position'''
        if np.linalg.norm(self.goal - self.y0) < 1e-4:
            print("Warning: Goal is too close to the start position. Adjusting goal slightly.")
            self.goal += 1e-4 * np.sign(self.goal - self.y0 + 1e-4)
    
    def imitate_path(self, y_des:np.array, plot=False):
        """Takes the desired path and computes the parameters to follow it
        y_des: desired path (n_dmps x timesteps)
        """
        # set initial state and goal
        # print(y_des)
        # print(y_des.ndim)
        if y_des.ndim == 1:
            y_des = y_des.reshape(1, -1)
        # print(y_des)
        # print(f"Shape of the desired path {y_des.shape}")
        # print(f"Desired path start: {y_des[:,0]}, end: {y_des[:,-1]}")
        self.y0 = y_des[:, 0].copy()
        self.y_des = y_des.copy()
        self.goal = self.gen_goal(y_des)
        # self.check_offset()
        
        # generate function to interpolate the desired trajectory
        import scipy.interpolate
        
        path = np.zeros((self.n_dmps, self.timesteps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1])
        for d in range(self.n_dmps):
            path_gen = scipy.interpolate.interp1d(x, y_des[d, :])
            for t in range(self.timesteps):
                path[d, t] = path_gen(t * self.dt)

        y_des = path
        
        # calculate velocity of y_des with central diffrences
        dy_des = np.gradient(y_des, axis=1) / self.dt
        
        # calculate acceleration of y_des with central diffrences
        ddy_des = np.gradient(dy_des, axis=1) / self.dt
        
        f_target = np.zeros((y_des.shape[1], self.n_dmps))
        
        #find the force required to move along this trajectory
        for d in range(self.n_dmps):
            f_target[:, d] = (ddy_des[d, :] - self.ay[d] * \
                (self.by[d] * (self.goal[d] - y_des[d, :]) - dy_des[d, :]))
            
        # generate weights to realize f_target
        self.gen_weights(f_target)
        
        if plot is True:
            # plot the basis function activations
            import matplotlib.pyplot as plt

            plt.figure()
            plt.subplot(211)
            psi_track = self.gen_psi(self.cs.rollout())
            plt.plot(psi_track)
            plt.title("basis functions")

            # plot the desired forcing function vs approx
            for ii in range(self.n_dmps):
                plt.subplot(2, self.n_dmps, self.n_dmps + 1 + ii)
                plt.plot(f_target[:, ii], "--", label="f_target %i" % ii)
            for ii in range(self.n_dmps):
                plt.subplot(2, self.n_dmps, self.n_dmps + 1 + ii)
                plt.plot(
                    np.sum(psi_track * self.w[ii], axis=1) * self.dt,
                    label="w*psi %i" % ii,
                )
                plt.legend()
            plt.title("DMP forcing function")
            plt.tight_layout()
            plt.show()

        self.reset_state()
        return y_des
    
    def rollout(self, timesteps=None, **kwargs):
        '''Generate a system trial without feedback'''
        
        self.reset_state()
        
        if timesteps is None:
            if 'tau' in kwargs:
                tau = kwargs['tau']
                timesteps = int(self.timesteps / tau)
            else:
                timesteps = self.timesteps
                
        # set up tracking variables
        y_track = np.zeros((timesteps, self.n_dmps))
        dy_track = np.zeros((timesteps, self.n_dmps))
        ddy_track = np.zeros((timesteps, self.n_dmps))
        
        for t in range(timesteps):
            y_track[t], dy_track[t], ddy_track[t] = self.step(**kwargs)

        return y_track, dy_track, ddy_track

    def step(self, tau=1.0, error=0.0, external_force=None, alpha_e=1.0, **kwargs):
        """Run the DMP system for a single timestep.

        tau float: scales the timestep
                   increase tau to make the system execute faster
        error float: optional system feedback
        """
        error_coupling = 1.0 / (1.0 + alpha_e * error)
        # run canonical system step
        x = self.cs.step(tau=tau, error=error)
        
        # generate basis function activations
        psi = self.gen_psi(x)
        
        for d in range(self.n_dmps):
            # generate the forcing term
            f = self.gen_front_term(x, d) * (np.dot(psi, self.w[d]))  
            sum_psi = np.sum(psi) + 1e-10  # avoid division by zero
            if sum_psi > 1e-6:
                f /= sum_psi
                
            # DMP acceleration
            self.ddy[d] = (self.ay[d] * (self.by[d] * (self.goal[d] - self.y[d]) - self.dy[d]) + f)
            
            if external_force is not None:
                self.ddy[d] += external_force[d]  # add external force if provided
            self.dy[d] += self.ddy[d] * error_coupling * tau * self.dt
            self.y[d] += self.dy[d] * error_coupling * tau * self.dt

        return self.y, self.dy, self.ddy

    def gen_front_term(self, x, dmp_num):
        raise NotImplementedError("Function gen_front_term needs to be implemented.")
    
    def gen_goal(self, y_des):
        raise NotImplementedError("Function gen_goal needs to be implemented.")
    
    def gen_psi(self):
        raise NotImplementedError("Function gen_psi needs to be implemented.")

    def gen_weights(self, f_target, x_track):
        raise NotImplementedError("Function gen_weights needs to be implemented.")