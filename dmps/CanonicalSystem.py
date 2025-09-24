from os import times
import numpy as np

class CanonicalSystem:
    def __init__(self, alpha_x=1.0, dt=0.01, pattern='discrete', T=1.0):
        '''
        Establishes a canonical system for Dynamic Movement Primitives (DMPs).
        Parameters:
        - alpha_x: Decay rate of the phase variable.
        - dt: Time step for integration.
        - pattern: 'discrete' for point-to-point movements, 'rhythmic' for cyclic movements.
        '''
        self.alpha_x = alpha_x  # Decay rate
        self.dt = dt        # Time step
        assert pattern in ['discrete', 'rhythmic'], "Pattern must be 'discrete' or 'rhythmic'"
        self.pattern = pattern  # 'discrete' or 'rhythmic'
        if pattern == 'discrete':
            self.step = self.step_discrete
            self.run_time = T
        else:
            self.step = self.step_rhythmic
            self.run_time = T * 2 * np.pi
        
        self.timesteps = int(self.run_time / self.dt)
        
        self.reset()

    def reset(self):
        '''reset the canonical system to initial state'''
        self.x = 1.0  # Initial phase variable

    def rollout(self, **kwargs):
        '''generate a rollout for the canonical system in open loop'''
        if 'tau' in kwargs:
            tau = kwargs['tau']
            timesteps = int(self.timesteps / tau)
        else:
            tau = 1.0
            timesteps = self.timesteps
        self.x_track = np.zeros(timesteps)
        
        self.reset()
        
        for t in range(timesteps):
            self.x_track[t] = self.x
            self.step(**kwargs)
        
        return self.x_track

    def step_discrete(self, tau=1.0, error=0.0, alpha_e=1.0):
        '''step function for discrete canonical system
        decaying from 1 to 0 following 1/tau * dx = -alpha_x * x
        parameters:
        - tau: temporal scaling factor (1/time_constant) higher values make the movement faster
        - error: coupling term to modulate the phase variable based
                        on external feedback (slow down if the error > 1)
        '''
        dx = (-self.alpha_x * self.x * (1/(1 + alpha_e * error) * tau))
        self.x += dx * self.dt
        return self.x

    def step_rhythmic(self, tau=1.0, error=0.0, alpha_e=1.0):
        '''step function for rhythmic canonical system
        oscillating between 1 and -1 following 1/tau * dx = -alpha_x * (x^2 + 1) * x
        parameters:
        - tau: temporal scaling factor (1/time_constant) higher values make the movement faster
        - error: coupling term to modulate the phase variable based
                        on external feedback (slow down if the error > 1)
        '''
        error_coupling = 1.0 / (1.0 + alpha_e * error)
        dx = (self.alpha_x * error_coupling * tau)
        self.x += dx * self.dt
        return self.x
    
# ==============================================
# Test the Canonical System
# ==============================================
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    dt = 0.001
    T = 1.0
    # Test discrete canonical system
    cs = CanonicalSystem(pattern='discrete', dt=dt, T=T)
    # get nominal rollout
    x_track_nominal = cs.rollout()
    
    cs.reset()
    
    # non-nominal rollout with error coupling
    timesteps = int(T/dt)
    x_track_coupled = np.zeros(timesteps)
    error = np.zeros(timesteps)
    error[200:400] = 2.0  # Simulate an error signal
    for t in range(timesteps):
        x_track_coupled[t] = cs.step(tau=1.0, error=error[t])

    t = np.arange(0, timesteps)*dt
    fig, ax1 = plt.subplots(figsize=(6, 3))
    ax1.plot(t, x_track_nominal, lw=2)
    ax1.plot(t, x_track_coupled, lw=2)
    plt.grid()
    plt.ylabel("x")
    plt.legend(["normal rollout", "error coupling"])
    ax2 = ax1.twinx()
    ax2.plot(t, error, "r-", lw=2)
    plt.legend(["error"], loc="lower right")
    plt.ylim(0, 3.5)
    plt.xlabel("time (s)")
    plt.ylabel("error")
    plt.title("Canonical system - discrete")

    for t1 in ax2.get_yticklabels():
        t1.set_color("r")

    plt.tight_layout()
    plt.show()