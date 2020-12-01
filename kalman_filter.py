import numpy as np
from filterpy.kalman import KalmanFilter as kalmanfilter

class KalmanFilter:
    def __init__(self, variance):
        self.kf = kalmanfilter(dim_x=4, dim_z=2)
        self.variance = variance
        # BEGIN_YOUR_CODE
        raise NotImplementedError
        # END_YOUR_CODE

    def get_x(self, meas):
        # BEGIN_YOUR_CODE
        raise NotImplementedError
        # END_YOUR_CODE
    