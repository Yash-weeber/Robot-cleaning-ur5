from .dmp import DMPs  # Import the base DMPs class from the dmps.dmp module
import numpy as np         # Import numpy for numerical operations

class DMPs_rhythmic(DMPs):  # Define a class for rhythmic DMPs, inheriting from DMPs
    """
    Rhythmic Dynamic Movement Primitives (DMPs) implementation.

    This class extends the base DMPs class to model rhythmic movements,
    such as walking or cycling, using Gaussian basis functions.
    It provides methods for generating basis function centers, computing
    basis function activations, and determining the goal for path imitation.
    """

    def __init__(self, **kwargs):
        '''
        Initialize a rhythmic DMPs system.

        Parameters
        ----------
        **kwargs : dict
            Additional keyword arguments passed to the base DMPs class.
            Common arguments include:
                n_dmps : int
                    Number of DMPs (dimensions).
                n_bfs : int
                    Number of basis functions.
                dt : float
                    Time step for integration.
                w : np.ndarray
                    Initial weights for the forcing term.

        Sets up the Gaussian basis function centers and variances,
        and checks for any required offset in the DMP system.
        '''
        super().__init__(pattern='rhythmic', **kwargs)  # Initialize the parent DMPs class with 'rhythmic' pattern
        
        self.gen_centers()  # Generate the centers for the Gaussian basis functions
        
        # set variance of Guassian basis functions
        self.h = np.ones(self.n_bfs) * self.n_bfs  # Set variance for each basis function
        
        self.check_offset()  # Check and set the offset for the DMP system
        
    def gen_centers(self):
        """
        Generate the centers of the Gaussian basis functions.

        The centers are spaced evenly throughout one cycle of the rhythmic movement,
        specifically over the interval [0, 2π].

        Returns
        -------
        None
        """
        c = np.linspace(0, 2 * np.pi, self.n_bfs + 1)  # Create evenly spaced points over [0, 2π]
        c = c[0:-1]  # Exclude the last point to avoid duplication at 0 and 2π
        self.c = c  # Assign centers to the instance variable

    def gen_front_term(self, x, dmp_num):
        """
        Generate the front term for the forcing term.

        For rhythmic DMPs, this term is non-diminishing. Thus, this function simply returns 1.
        """
        if isinstance(x, np.ndarray):
            return np.ones(x.shape)  # Return an array of ones if x is an array
        return 1  # Return 1 
    
    def gen_goal(self, y_des):
        """
        Generate the goal for path imitation.

        For rhythmic DMPs, the goal is defined as the average of the desired trajectory,
        calculated as the midpoint between the minimum and maximum values of the trajectory.

        Parameters
        ----------
        y_des : np.ndarray
            The desired trajectory to follow, with shape (n_dmps, n_timesteps).

        Returns
        -------
        goal : np.ndarray
            The computed goal for each DMP, with shape (n_dmps,).
        """
        goal = np.zeros(self.n_dmps)  # Initialize the goal array
        for n in range(self.n_dmps):
            num_idx = ~np.isnan(y_des[n])  # Ignore NaN values when calculating the goal
            goal[n] = 0.5 * (y_des[n, num_idx].min() + y_des[n, num_idx].max())  # Compute the average of min and max

        return goal  # Return the computed goal
    
    def gen_psi(self, x):
        """
        Generate the activity of the basis functions for a given
        canonical system state or path.

        Parameters
        ----------
        x : float or np.ndarray
            The canonical system state or path.

        Returns
        -------
        psi : np.ndarray
            The activations of the basis functions, with shape (len(x), n_bfs) if x is an array,
            or (n_bfs,) if x is a scalar.
        """
        if isinstance(x, np.ndarray):
            x = x[:, None]  # Reshape x to be a column vector for broadcasting

        # Compute the Sinusoidal basis function activations
        psi = np.exp(self.h * (np.cos(x - self.c) - 1))

        return psi  # Return the computed basis function activations
    
    def gen_weights(self, f_target):
        """
        Generate a set of weights over the basis functions such
        that the target forcing term trajectory is matched.

        Parameters
        ----------
        f_target : np.ndarray
            The desired forcing term trajectory, with shape (n_timesteps, n_dmps).

        Returns
        -------
        None
        """
        # calculate x and psi
        x_track = self.cs.rollout()  # Roll out the canonical system to get the phase variable over time
        psi_track = self.gen_psi(x_track)  # Compute the basis function activations over time

        # calculate BF weights using weighted linear regression
        for d in range(self.n_dmps):
            for b in range(self.n_bfs):
                self.w[d, b] = np.dot(psi_track[:, b], f_target[:, d]) / (
                    np.sum(psi_track[:, b]) + 1e-10  # Avoid division by zero
                )  # Compute the weight for each basis function and DMP dimension