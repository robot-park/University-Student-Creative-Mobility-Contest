from math import sin, degrees, atan2, radians
from numpy import clip, clip
import rospy
from visualization_msgs.msg import Marker, MarkerArray
class LatController():
    def __init__(self, eg, sh, pl, park):
        self.dt = 0.1
        self.k = 0.5 # Control gain
        self.ego = eg
        self.plan = pl
        self.park = park
        self.shared = sh
        self.global_path = self.shared.global_path
        self.local_path = self.shared.local_path
        self.WB = 1.04
        self.length = 1.6
        self.width = 1.16
        self.backtowheel = 0.3
        self.Wheel_len = 0.33
        self.wheel_width = 0.175
        self.TREAD = 0.545
        self.yaw = self.ego.heading
        self.steer = 0
        self.state = None
        self.target_x = 0 
        self.target_y = 0
        self.pub = rospy.Publisher("/target_point", MarkerArray, queue_size=1)
        self.msg = Marker()
        self.marker_array = MarkerArray()
        self.prev = 0
    def run(self):
        while True:
            try:

                if self.shared.plan.motion_decision == "Test":
                    return self.global_stanley_method()
                
                elif self.shared.plan.motion_decision == "obs_small":
                    return self.local_stanley_method()
                
                elif self.shared.plan.motion_decision == "obs_big":
                    return self.local_stanley_method()
                
                elif self.shared.parallel_local_control:
                    return self.local_stanley_method()
                
                elif self.shared.manual_steer == True:
                    return self.manually_steer()
                
                else:
                    return self.global_stanley_method()
                
            except IndexError:
                # print("++++++++lat_controller+++++++++")
                pass
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
  
    def global_stanly_method(self):
        self.state ="stanly_method"
        yaw = self.ego.heading
        x = self.ego.x
        y = self.ego.y
        mind_dist = 1e9
        min_index = 0
        self.path = self.shared.global_path
        n_points = len(self.path.x)
        
        front_x = x + self.WB * np.cos(yaw)
        front_y = y + self.WB * np.sin(yaw)
        
        if self.ego.target_gear == 2:
            yaw = (yaw - 180) % 360
        # 가장 가까운 인덱스를 찾는 loop문
        for i in range(n_points):
            dx = front_x - self.path.x[i]
            dy = frony_y - self.path.y[i] 
            
            dist = np.sqrt(dx * dx + dy* dy)
            if dist < min_dist:
                min_dist = dist
                min_index = i
				min_index, min_dist 
        

        map_x2 = self.path.x[min_index+1] # for map_yaw
        map_x = self.path.x[min_index]
        map_y2 = self.path.y[min_index+1] # for map_yaw
        map_y = self.path.y[min_index]
        
        map_yaw =  atan2(map_x2-map_x,map_y2-map_y)  
        dx = map_x - front_x
        dy = map_y - front_y
        
        perp_vec = [np.cos(yaw + np.pi/2),np.sin(yaw + np.pi/2)]
        cte = np.dot([dx,dy], perp_vec)
				# cte = lateral distance 

        yaw_term = normalize_angle(map_yaw - yaw)
        cte_term = np.arctan2(self.k*cte,self.gspeed)
				#cte_term = np.arctan2(self.k1*cte,self.k2+self.gspeed)
				#self.gspeed = 현재 속도? 
        
        tmp_steer = yaw_term + cte_term
        self.steer = float(clip(-tmp_steer,-27,27))
        return self.steer
    
    def local_stanly_method(self):
        self.state ="stanly_method"
        yaw = self.ego.heading
        x = self.ego.x
        y = self.ego.y
        mind_dist = 1e9
        min_index = 0
        self.path = self.shared.local_path
        n_points = len(self.path.x)
        
        front_x = x + self.WB * np.cos(yaw)
        front_y = y + self.WB * np.sin(yaw)
        
        if self.ego.target_gear == 2:
            yaw = (yaw - 180) % 360
            
        
        for i in range(n_points):
            dx = front_x - self.path.x[i]
            dy = frony_y - self.path.y[i] 
            
            dist = np.sqrt(dx * dx + dy* dy)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        
        map_x2 = self.path.x[min_index+1] # for map_yaw
        map_x = self.path.x[min_index]
        map_y2 = self.path.y[min_index+1] # for map_yaw
        map_y = self.path.y[min_index]
        
        map_yaw =  atan2(map_x2-map_x,map_y2-map_y)# 이게 뭐지
        
        dx = map_x - front_x
        dy = map_y - front_y
        
        perp_vec = [np.cos(yaw + np.pi/2),np.sin(yaw + np.pi/2)]
        cte = np.dot([dx,dy], perp_vec) 
        
        yaw_term = normalize_angle(map_yaw - yaw)
        cte_term = np.arctan2(self.k*cte,self.gspeed)
        
        tmp_steer = yaw_term + cte_term
        self.steer = float(clip(-tmp_steer,-27,27))
        return self.steer
    def manually_steer(self):
        self.state = "manual steer"
        return self.ego.target_steer