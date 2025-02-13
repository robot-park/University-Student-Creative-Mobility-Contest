from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from collections import deque
import numpy as np

class Object():

    def __init__(self, bbox):
        self.bbox = bbox
        self.dt = 0.1
        self.ukf_x = None
        self.ukf_y = None
        self.count = 1

    # def fx(self, x, dt, **kwargs):
    #     return np.array([x[0] + dt*x[1], x[1]])

    # def fx(self, x):
    #     return np.array([x[0] + self.dt*x[1], x[1]])
    def fx(self, x, dt):
        F = np.array([[1, dt],
                       [0, 1]], dtype=float)
        return np.dot(F, x)

    def hx(self, x):
        return np.array([x[0]])


    def sigmapoints(self):
        points = MerweScaledSigmaPoints(2, alpha=1e3, beta=2., kappa=1.)
        return points
    

    def set_initial_value(self, z):
        points = self.sigmapoints()
        self.ukf_x = UnscentedKalmanFilter(dim_x=2, dim_z=1, dt=0.1,
                            fx=self.fx, hx=self.hx,
                            points=points)
        self.ukf_y = UnscentedKalmanFilter(dim_x=2, dim_z=1, dt=0.1,
                            fx=self.fx, hx=self.hx,
                            points=points)
        self.ukf_x.x = np.array([z[0], 0.])
        self.ukf_y.x = np.array([z[1], 0.])

        # 오차 공분산 from sensor
        self.ukf_x.R = np.array([0.0001])   
        self.ukf_y.R = np.array([0.0001])     

        #오차 공분산 from model
        self.ukf_x.Q = np.array([3.1])
        self.ukf_y.Q = np.array([3.1])


    def Predict(self):
        if self.ukf_x is not None and self.ukf_y is not None:    
            self.ukf_x.predict()
            self.ukf_y.predict()
        else:
            print("can't predict nan value")

    def Update(self, z, bbox):
        if self.ukf_x is not None and self.ukf_y is not None:    
            z_x = np.array([z[0]]).T
            z_y = np.array([z[1]]).T
            self.ukf_x.update(z_x)
            self.ukf_y.update(z_y)
            self.bbox = bbox
            self.bbox[0] = self.ukf_x.x[0]
            self.bbox[1] = self.ukf_y.x[0]

    def getVelocity(self):
        return self.ukf_x.x[1], self.ukf_y.x[1]