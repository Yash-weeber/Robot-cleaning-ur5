from .dmp import DMPs  # Import the base DMPs class from the dmps.dmp module
import numpy as np         # Import numpy for numerical operations

class DMPs_discrete(DMPs):  # Define a class for discrete DMPs, inheriting from DMPs
    """
    Discrete Dynamic Movement Primitives (DMPs) implementation.

    This class extends the base DMPs class to model discrete movements,
    such as reaching or pointing, using Gaussian basis functions.
    It provides methods for generating basis function centers, computing
    basis function activations, and fitting the forcing term weights via
    linear regression.
    """

    def __init__(self, **kwargs):
        '''
        Initialize a discrete DMPs system.

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
        super().__init__(pattern='discrete', **kwargs)  # Initialize the parent DMPs class with 'discrete' pattern
        
        self.gen_centers()  # Generate the centers for the Gaussian basis functions
        
        # set variance of Guassian basis functions
        self.h = np.ones(self.n_bfs) * self.n_bfs**1.5 / self.c / self.cs.alpha_x  # Calculate the variance for each basis function
        
        self.check_offset()  # Check and set the offset for the DMP system
        
    def gen_centers(self):
        """
        Generate the centers of the Gaussian basis functions.

        The centers are spaced evenly throughout the run time of the canonical system,
        and then mapped to the canonical system's phase variable using an exponential decay.

        Returns
        -------
        None
        """
        # desired activations throughout time
        des_c = np.linspace(0, self.cs.run_time, self.n_bfs)  # Create evenly spaced time points for centers

        self.c = np.ones(len(des_c))  # Initialize centers array
        for n in range(len(des_c)):
            # finding x for desired times t
            self.c[n] = np.exp(-self.cs.alpha_x * des_c[n])  # Compute the center for each basis function
            
    def gen_front_term(self, x, dmp_num):
        """
        Compute the front term for the forcing function.

        For discrete DMPs, this term scales the forcing function by the distance
        between the goal and the initial position.

        Parameters
        ----------
        x : float or np.ndarray
            Canonical system state or phase variable.
        dmp_num : int
            Index of the DMP dimension.

        Returns
        -------
        float or np.ndarray
            The front term value.
        """
        return x * (self.goal[dmp_num] - self.y0[dmp_num])  # Compute the front term for the forcing function
    
    def gen_goal(self, y_des):
        """
        Extract the goal from the desired trajectory.

        For discrete DMPs, the goal is set as the final value of the desired trajectory.

        Parameters
        ----------
        y_des : np.ndarray
            Desired trajectory, shape (n_dmps, T).

        Returns
        -------
        np.ndarray
            Goal for each DMP dimension.
        """
        return np.copy(y_des[:, -1])  # Set the goal as the last value of the desired trajectory

    def gen_psi(self, x):
        """
        Compute the activation of the Gaussian basis functions.

        Parameters
        ----------
        x : float or np.ndarray
            Canonical system state or phase variable.

        Returns
        -------
        np.ndarray
            Basis function activations, shape (len(x), n_bfs).
        """
        if isinstance(x, np.ndarray):  # If x is an array
            x = x[:, None]             # Reshape x to a column vector
        # calculate the activation of the basis functions
        psi = np.exp(-self.h * (x - self.c)**2)  # Compute the Gaussian activation for each basis function
        return psi  # Return the activation values
    
    def gen_weights(self, f_target):
        """
        Fit the weights for the forcing term using weighted linear regression.

        Parameters
        ----------
        f_target : np.ndarray
            Target forcing term, shape (T, n_dmps).

        Returns
        -------
        None
        """
        x_track = self.cs.rollout()  # Get the canonical system rollout (phase variable trajectory)

        psi_track = self.gen_psi(x_track)  # Compute the basis function activations for the rollout
        
        # calculate the basis function weights using weighted linear regression
        self.w = np.zeros((self.n_dmps, self.n_bfs))  # Initialize the weights array
        for d in range(self.n_dmps):  # Loop over each DMP dimension
            # spatial spacing
            k = self.goal[d] - self.y0[d]  # Compute the scaling factor for the movement
            for b in range(self.n_bfs):    # Loop over each basis function
                # weighted linear regression
                numer = np.sum(x_track * psi_track[:, b] * f_target[:, d])  # Numerator for regression
                denom = np.sum((x_track**2) * psi_track[:, b])              # Denominator for regression
                if denom > 0:
                    self.w[d, b] = numer / denom  # Compute the weight if denominator is positive
                else:
                    self.w[d, b] = 0.0            # Otherwise, set weight to zero
                if abs(k) > 1e-5:
                    self.w[d, b] /= k             # Normalize the weight by the movement scale if significant
