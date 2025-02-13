import numpy as np
import matplotlib.pyplot as plt
import rospy
import message_filters
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque
import math


# P 낮추고, R 크게 설정하여서 예측값에 대한 비중 높이기
# EKF 알고리즘
# for z in data:
    # 예측 단계
positions = []
reference = []
class Object:
    global positions, reference
    def __init__(self, bbox):
        self.bbox = bbox
        self.dt = 0.1 
        self.Q = np.diag([0.1, 0.1])  

        self.Rx = np.array([[2.5]]) 
        self.Ry = np.array([[0.5]])
        self.x_est = np.array([0.0, 0.0]) 
        self.y_est = np.array([0.0, 0,0]) 
        self.Px_est = np.diag([0.001, 0.001])  
        self.Py_est = np.diag([0.1,0.1]) 
        self.count = 0
        self.z_x = 0
        self.z_y = 0
        self.heading = deque(maxlen=5)
        self.velo_x = deque(maxlen=10)
        self.velo_y = deque(maxlen=10)
        self.collision_pos_y = 0
        self.appear = 0

    def set_initial_value(self, z_x, z_y):
        self.x_est = np.array([z_x, 0])
        self.y_est = np.array([z_y, 0])
        self.count += 1

    def predict(self):
        x_pred = self.x_est + np.array([self.x_est[1]*self.dt, 0.0])
        y_pred = self.y_est + np.array([self.y_est[1]*self.dt, 0.0])
        return x_pred, y_pred

    def update(self, z_x, z_y):
        self.z_x = z_x
        self.z_y = z_y
        x_pred = self.x_est + np.array([self.x_est[1]*self.dt, 0.0])
        y_pred = self.y_est + np.array([self.y_est[1]*self.dt, 0.0])

        F = np.array([[1.0, self.dt],[0.0, 1.0]])

        Px_pred = F.dot(self.Px_est).dot(F.T) + self.Q
        Py_pred = F.dot(self.Py_est).dot(F.T) + self.Q

        H = np.array([[1.0,0.0]])
        
        Sx = H.dot(Px_pred).dot(H.T) + self.Rx
        Sy = H.dot(Py_pred).dot(H.T) + self.Ry

        #normal form
        # Kx = Px_pred.dot(H.T).dot(np.linalg.solve(Sx, np.eye(1)))
        # Ky = Py_pred.dot(H.T).dot(np.linalg.solve(Sy, np.eye(1)))
        #Joseph Form
        Kx = Px_pred.dot(H.T).dot(np.linalg.inv(self.Rx + H.dot(Px_pred).dot(H.T)))
        Ky = Py_pred.dot(H.T).dot(np.linalg.inv(self.Ry + H.dot(Py_pred).dot(H.T)))
        Px_updated = (np.eye(Px_pred.shape[0]) - Kx.dot(H)).dot(Px_pred).dot((np.eye(Px_pred.shape[0]) - Kx.dot(H)).T) + Kx.dot(self.Rx).dot(Kx.T)
        Py_updated = (np.eye(Py_pred.shape[0]) - Ky.dot(H)).dot(Py_pred).dot((np.eye(Py_pred.shape[0]) - Ky.dot(H)).T) + Ky.dot(self.Ry).dot(Ky.T)

        if abs(self.z_y - self.y_est[0]) > 0.6:
            x_est = x_pred + Px_pred.dot(H.T).dot(np.linalg.pinv(self.Rx + H.dot(Px_pred).dot(H.T))).dot(self.z_x - H.dot(x_pred))
            
            self.x_est = x_est
            self.Px_est = Px_updated
            self.y_est = y_pred
            self.Py_est = Py_pred

        else: 
            x_est = x_pred + Px_pred.dot(H.T).dot(np.linalg.pinv(self.Rx + H.dot(Px_pred).dot(H.T))).dot(self.z_x - H.dot(x_pred))
            y_est = y_pred + Py_pred.dot(H.T).dot(np.linalg.pinv(self.Ry + H.dot(Py_pred).dot(H.T))).dot(self.z_y - H.dot(y_pred))

            self.x_est = x_est
            self.Px_est = Px_updated
            self.y_est = y_est
            self.Py_est = Py_updated

        self.appear +=1

    def future_point(self):      
        forward = []
        new_collision_pos_y = 0
        for i in range(5):
            x = self.x_est + np.array([self.x_est[1] * 0.5 * i, 0.0])
            y = self.y_est + np.array([self.y_est[1] * 0.5 * i, 0.0])
            forward.append([x[0],y[0]])
        collision_time = self.x_est[0]/self.x_est[1]
        new_collision_pos_y = np.round((self.y_est[0] - self.y_est[1]*collision_time),3)
        

        if abs(self.x_est[1]) < 0.2:
            pass
        else:
            self.collision_pos_y = new_collision_pos_y
        self.appear +=1    
        
        return forward