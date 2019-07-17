#!/usr/bin/env python
from numpy import *

class Filter():
    """
    Performs Kalman Filter state estimation on data provided by the sensors. Data units
    are in the Standard Unit system (e.g. meters, seconds, rads).

    System Attributes
    ----------------
    Delta_T: float.
        Interval of time between estimations, in seconds.

    State Attributes
    ----------------
    S: float. 
        Position of the robot on the road (1D), in meters.
    P: float.
        Variance of the position of the robot, in squared meters.

    Uncertainty Attributes
    ---------------
    sigma_L: float.
        The standard deviation associated to the position of the landmark in the environment, in meters.
    sigma_omega: float.
        The standard deviation associated to the depth measured by the sensor, in meters.
    R: float.
        The variance of z(X), in squared meters.

    Extended State Attributes
    ---------------
    d_x: float.
        The sum of displacements between landmarks, in meters.
    direction: int.
        Indicates the direction of the motion. If `direction=1`, the robot is moving
        from the origin to the end. If `direction=-1`, the robot is moving from the
        end to the origin of the road. If `direction=None` the direction was not defined
        yet.
    l: float.
        Position indicated on the most current detected sign, in meters.
    d_l: float.
        The projection of the landmark distance from the robot on the road, in meters.
    sigma_d_l: float.
        The deviation of the projected distance between the landmark and the robot, in meters.
    """

    def predict(self, v):
        """
        Estimates the next state given the measured velocity. Only possible after
        the first landmark is detected.

        Parameters
        -----------
        v: float.
            The measured velocity, in meters per second.

        Return
        -----------
        boolean: Whether it was possible to predict the state (`True`) or not (`False`).
        """
        # If there is no previous landmark detected
        if(self.l is None):
            return False
        # Displacement and uncertainty 
        sigma_Delta_v = (0.1 * v / 3.0) # The deviance is somewhere around 10% of the velocimeter value. 
        direction = 1.0
        if(self.direction is not None):
            direction = self.direction
        var_Delta_S = (self.Delta_T**2.0) * (sigma_Delta_v**2.0)
        delta_S =  v * self.Delta_T * direction
        # Estimation
        self.d_x += delta_S
        self.P = self.P + var_Delta_S
        if(self.direction is not None):
            self.S = self.S + delta_S
            return False
        return True

    def update(self, l, omega, psi):
        """
        Parameters
        -------------
        l: float.
            Position indicated on the detected landmark, in meters.
        omega: float.
            Distance measured from the sensor to the landmark, in meters.
        psi: float.
            Angle between the sensor's horizon and the landmark, in radians.
        """
        d_l = omega * cos(psi)
        sigma_d_l = self.sigma_omega * cos(psi)
        if(l is None):
            raise "No landmark was provided."
        if(self.l is None):
            self.S = l
            self.P = (self.sigma_L ** 2.0) + ( sigma_d_l ** 2.0)
            self.l = l
            self.d_l = d_l
            self.sigma_d_l = sigma_d_l
            return False
        if(self.direction is None):
            self.direction = sign(l - self.l)
            self.d_x *= self.direction
            self.S = self.S + self.d_x

        # Measurement and uncertainty handler
        z = l - d_l * self.direction
        h = self.S
        R = (self.sigma_L**2.0) + (sigma_d_l**2.0)

        # Kalman Update
        K = self.P / (self.P + R)
        innovation = (z - h)
        S_est = self.S + K * innovation
        P_est = (1 - K) * self.P

        # Variable replacement
        self.l = l
        self.d_l = d_l
        self.sigma_d_l = sigma_d_l
        self.S = S_est
        self.P = P_est
        self.d_x = 0
        return True

    def __init__(self):
        self.S = 0.0
        self.P = 1e6
        self.Delta_T = 1e-2
        #self.sigma_L = (100.0/3.0)
        self.sigma_L = 0
        self.sigma_omega = 1e-1
        self.clear()
        return
    
    def clear(self):
        self.d_x = 0.0
        self.l = None
        self.direction = None
        return 
