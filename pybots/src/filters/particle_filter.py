# -*- coding: utf-8 -*-
"""
Created on Sat Feb  7 15:18:06 2015

@author: Nate
"""
import numpy as np

class ParticleFilter(object):
    """Simple implementation of a particle filter with semi-intelligent 
    resample."""
    def __init__(self, sys_model, meas_model=None, num_particles=1000, 
                 initial_state=[0.0]):
        """Initialize particle filter class with system and measurement models 
        as well as optionally specifying the number of particls and the initial 
        state.
        
        Args: 
            sys_model: a function that takes an array of particles, Xt and 
                inputs Ut and returns Xt+1
            meas_model: a function that takes an array of particles, Xt and a 
                measurement, and returns an array of weights corresponding to 
                the input particles
            num_particles(optional): an integrer number of particles to be 
                projected
            initial_state(optional): an array indicating the initial state of 
                the system
        """
        self.M = num_particles # integer number of particles   
        self.initialize_filter(initial_state) # set of particles
        #self.w = np.ones(self.M) # weight associated with each particle
        self._sys_model = sys_model # system model function
        self._meas_model = meas_model # measurement model function
        self._resample_test = resample_when_skewed(1.2) # a resample test
        
    def initialize_filter(self, initial_state=None, noise_array=0.0):
        """Method initializes the member self.X
        
        Args:
            initial_state: a list indicating the initial system state
        """
        if initial_state is not None:
            # save a copy of the filter's initial state
            self.x0 = initial_state
        else:
            # otherwise, reset the filter
            initial_state = self.x0
        # set the array of particles and weights
        self.X = np.tile(initial_state, [self.M, 1])
        self.w = np.ones(self.M)
    
    def sample(self, u):
        """Method samples given the system model and the input and advances 
        belief state.
        
        Args:
            u: control input at the current timestep (should match expected 
                input in system_model)
                
        Returns:
            a copy of st state array in case its needed by anyone
        """
        self.X = self._sys_model(self.X, u, self.M)
        # return in case anyone cares
        return self.X
        
    def assign_weight(self, z):
        """Method assigns an importance factor to each candidate state.
        
        Args:
            z: measurement at the current timestep, passed directly to the 
                measurement model
        """
        self.w = np.multiply(self.w, self._meas_model(self.X, z))
        
    def resample(self):
        """Method checks the resample criteria (skewedness of distribution) and 
        resamples the particles if necessary.
        
        No Arguments.
        """
        w = self.normalize_weight() # normalize the particle weights
        if self._resample_test(w):
            self.low_var_resample(w)
            
    def expectation(self):
        """Method returns the expectation of the state.
            
        Returns:
            E(X): scalar expected value of the distribution
        """
        w = self.normalize_weight()  
        return [np.multiply(self.X[:,i],w).sum() for i in range(self.X.shape[1])]
            
    def normalize_weight(self):
        """Method returns the normalize weight array of the object.
        
        No Arguments        
        """
        if self.w.sum() == 0.0:
            # if all particles have a weight of zero.. there is an issue
            return np.ones(self.M)/self.M
        elif self.w.sum() != 1.0:
            # if the sum of the weights is something other than 1.0, normalize
            return self.w/self.w.sum()
        else:
            # if the sum of the weights is 1.0, nothing need be done.
            return self.w
    
    def low_var_resample(self, w):
        """Method implements a low variance deterministic sampler as described 
        in Thrun pg. 110.
        
        Args:
            w: associated weights normalized such that they sum to 1.0 (M)
        Returns:
            X: a set of particles representing the belief at t
        """
        # low_variance_sampler from Thrun pg. 110
        X = np.zeros(self.X.shape)
        r = np.random.uniform(0.0, 1.0/self.M)
        c = w[0]
        i = 0
        for m in range(self.M):
            U = r + (m)*(1.0/self.M)
            while U > c:
                i = i+1
                c = c + w[i]
            X[m] = self.X[i]
        # save the new beleif
        self.X = X
        # reset the weights
        self.w = np.ones(self.M)
        
def resample_when_skewed(threshold):
    '''Resample when the weights in a particle filter are too skewed.
    threshold: A numeric threshold that determines the tolerated skew above
    which particles should be resampled. If this is around 2, resampling
    occurs whenever about 25 % of the particles have about 75 % of the weight.
    Other easily interpreted thresholds are :
        50/50  1.0000 (i.e. resample wih every observation)
        60/40  1.1667
        70/30  1.7619
        80/20  3.2500
        90/10  8.1111
    '''
    # note, lambda functions are not needed...
    return lambda w: (len(w) * (w * w).sum()) > threshold