#!/usr/bin/env python
from numpy import *

class Filter():
    """
    Performs Kalman Filter state estimation on data provided by the sensors. Data units
    are in the Standard Unit system (e.g. meters, seconds, rads).

    State Attributes
    ----------------
    X: ndarray. 
        The vectorized state of the robot.
    P: ndarray.
        Covariance of the state of the robot.
    """

    def predict(self, A, B, u, Q):
        """
        Estimates the next state given the motion model matrices `A` and `B` 
        and the control command `u`.
        """
        self.X = A.dot(self.X) + B.dot(u)
        self.P = (A.dot(self.P)).dot(A.transpose()) + Q
        return

    def update(self, z, h, H, R):
        """
        Parameters
        --------------------
        z:  float.
            Measurement that corrects the estimation.
        h:  function.
            Returns the transformation of the state to the measurement space.
        H:  ndarray.
        R:  ndarray.
            Covariance matrix of the measurement process.
        """
        # Kalman Update
        factor = self.P.dot(H.transpose())
        K = factor.dot(  
            linalg.inv(H.dot(factor) + R)
        )
        innovation = ( z - h(self.X) )
        
        X_est = self.X + K.dot(innovation)
        P_est = (eye(2) - K.dot(H)).dot(self.P)
        self.X = X_est
        self.P = P_est
        return

    def covariance(self):
        return self.P

    def state(self):
        return self.X

    def __init__(self, X_init, P_init):
        self.X = X_init
        self.P = P_init
        return
