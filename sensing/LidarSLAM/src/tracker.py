#여기에 추가를 할 것이 뭐냐면 ekf를 사용해서 차량의 다음 거동을 예측하는 거지 예측을 한 후에 만약에 차량이 사라졌으면 그 예측된 경로에다가 하나를 띄우는 과정이 필요함.
#이 과정까지 한다면 신호처리 방식으로 라이다를 사용하는 것은 거의 끝이 아닌가 싶음.
#센서 퓨전을 얹어서 하자
#단 calibration 다시 해보자 이상함.

from EKF_joseph import Object
from numpy.linalg import inv
from visualization_msgs.msg import Marker, MarkerArray
from scipy.optimize import linear_sum_assignment
from collections import deque
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseArray
import copy
import time
# import cv2
import numpy as np
import rospy
import math
import datetime as dt
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
from message_filters import ApproximateTimeSynchronizer, Subscriber
import message_filters
from geometry_msgs.msg import Point
from collections import deque



dk = 0.5
A_k = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
H_k = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
Q_k = np.array([[1.0,0,0],[0,1.0,0],[0,0,1.0]])
R_k = np.array([[0.001,0,0],[0,0.001,0],[0,0,0.001]])
w_k = np.array([0.07,0.01,0.04])
noise = np.array([0.01,0.01,0.01])

 
def quaternion_from_euler(yaw):
    qx = np.sin(0) * np.cos(0) * np.cos(yaw/2) - np.cos(0) * np.sin(0) * np.sin(yaw/2)
    qy = np.cos(0) * np.sin(0) * np.cos(yaw/2) + np.sin(0) * np.cos(0) * np.sin(yaw/2)
    qz = np.cos(0) * np.cos(0) * np.sin(yaw/2) - np.sin(0) * np.sin(0) * np.cos(yaw/2)
    qw = np.cos(0) * np.cos(0) * np.cos(yaw/2) + np.sin(0) * np.sin(0) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def euler_from_quaternion(qx, qy, qz, qw):
    
    t1 = +2.0 * (qw * qz + qx * qy)
    t2 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_z = math.atan2(t1, t2)
    
    return yaw_z


def velocity(velo, diff):
    velo.append(diff)
    if len(velo) == 10:
        mean_velo = (sum([i for i in velo])/len(velo))
    else:
        mean_velo = 0
    return mean_velo


class ObjectTracker:
    def __init__(self,  remove_count=25, cost_threshold=0.5, appear_count=3):
        self.next_object_id = 0
        self.prev_objects = {}
        self.disappeared = {}
        self.appeared = {}
        self.remove_count = remove_count
        self.cost_threshold = cost_threshold
        self.appear_count = appear_count
        rospy.Subscriber('/markers', MarkerArray, self.callback)
        self.tracker_pub = rospy.Publisher("/tracked_marker", MarkerArray, queue_size=10)    
        self.footprint_pub = rospy.Publisher("/foot_print", MarkerArray, queue_size=10) #오는 길에 총총총총 남기고 싶었으나 ros 메시지는 하나씩 밖에 쏠 수 없음. 따라서 여러개를 쏴야 총총총 남을텐데 굳이?
        self.text_pub = rospy.Publisher("/text", MarkerArray, queue_size=10)
        self.dong_jang_feel_pub = rospy.Publisher('/dong_jang_feel', PoseArray, queue_size=10)
        self.x_list = []
        self.y_list = []
        self.i = 0
        self.markerarray = None

    def callback(self, msg):
        self.markerarray = msg

    def transfer(self, erp_x,erp_y,heading, object_x, object_y):
        rotated_x = (object_x+1.33)*math.sin(heading) + object_y*math.cos(heading)
        rotated_y = (object_x+1.33)*math.cos(heading) + object_y*math.sin(heading)

        enu_pos_x = rotated_x + erp_x
        enu_pos_y = rotated_y + erp_y
        return enu_pos_x, enu_pos_y

    def check(self, count):
        # tmp_count new object
        # 이미 있는 거랑 임시로 숫자 늘려놓은 거랑
        # 몇 프레임 이상 들어오면 그 땐 ㄹㅇ객체로 추가.
        if count != len(self.appeared):
            self.appeared[count] = 0
        else:
            self.appeared[count] +=1

    def push_back(self, object): #일단 추가는 했는데 다음 프레임에서도 추가가 되어야 이 객체를 유지할 것 아냐 맞지. 
        #the type of tracking is Kalmanfilter object
        self.prev_objects[self.next_object_id] = object
        # self.prev_objects[self.next_object_id].P_k = P_k
        # self.prev_objects[self.next_object_id].age = 0
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1
        print("object update")

    def remove(self, object_id):
        del self.prev_objects[object_id]
        del self.disappeared[object_id]

    def cost(self, prev_object, bbox): #객체사이의 거리를 기반으로 이게 같은 물체인지 tracking을 할 예정임. -> 추후에 같은 id인 객체의 값이 너무 튀는 경우가 발생하면 이를 보정하도록 하는 코드도 추가할 예정
        prev_center_x, prev_center_y, prev_center_z = prev_object.bbox[0], prev_object.bbox[1], prev_object.bbox[2] #이전 프레임에서의 객체의 가운데 좌표
        prev_size_x, prev_size_y, prev_size_z = prev_object.bbox[0], prev_object.bbox[1], prev_object.bbox[2]
        new_center_x, new_center_y, new_center_z = bbox.bbox[0], bbox.bbox[1], bbox.bbox[2] #현재 프레임에서 객체의 가운데 좌료
        new_size_x, new_size_y, new_size_z = bbox.bbox[0], bbox.bbox[1], bbox.bbox[2] #현재 프레임에서 객체의 가운데 좌료
        center_a = [prev_center_x, prev_center_y, prev_center_z]
        scale_a = [prev_size_x, prev_size_y, prev_size_z]
        center_b = [new_center_x, new_center_y, new_center_z]
        scale_b = [new_size_x, new_size_y, new_size_z]
        

        overlap = 1
        for i in range(3):
            length_a = scale_a[i] / 2
            length_b = scale_b[i] / 2

            left_a = center_a[i] - length_a
            right_a = center_a[i] + length_a

            left_b = center_b[i] - length_b
            right_b = center_b[i] + length_b

            overlap *= max(0, min(right_a, right_b) - max(left_a, left_b))

        
        distance = math.sqrt((prev_center_x - new_center_x)**2 + (prev_center_y - new_center_y)**2 + (prev_center_z - new_center_z)**2) 
        return distance

    def size(self, bbox):
        size = bbox.bbox[3] * bbox.bbox[4] * bbox.bbox[5]
        return size

    def main(self):
        if self.markerarray is not None:
            first = time.time()
            print("tracking")
            print(self.next_object_id)
            bboxes = []
            for marker in self.markerarray.markers:
                center_x = marker.pose.position.x 
                center_y = marker.pose.position.y
                center_z = marker.pose.position.z
                scale_x = marker.scale.x
                scale_y = marker.scale.y
                scale_z = marker.scale.z
                qx = marker.pose.orientation.x
                qy = marker.pose.orientation.y
                qz = marker.pose.orientation.z
                qw = marker.pose.orientation.w
                yaw = euler_from_quaternion(qx,qy,qz,qw)
                bbox = [center_x, center_y, center_z, scale_x, scale_y, scale_z, qx,qy,qz,qw,yaw]
                object = Object(bbox)
                bboxes.append(object)

            # if len(self.prev_objects) > 0:
            object_ids = list(self.prev_objects.keys())
            cost_matrix = np.zeros((len(self.prev_objects), len(bboxes)))

            #얼라 이거 왜 실행 안되지
            for i in range(len(object_ids)):
                for j in range(len(bboxes)):
                    cost_matrix[i, j] = self.cost(self.prev_objects[object_ids[i]], bboxes[j]) #이걸 boundingbox의 겹치는 정도를 이용하자.
            
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            used_rows = set()
            used_cols = set()

            for row, col in zip(row_ind, col_ind): #만약에 둘 다 리스트가 존재 한다면? 즉 쌍이 있다고 한다면
                if row in used_rows or col in used_cols: 
                    continue

                if cost_matrix[row, col] > 0.5:#self.cost_threshold : #그 거리가 일정 이상이 된다면? 제외하고 그 이하의 거리의 물체만 남기도록 하자
                    continue
                
                # if cost_matrix[row, col] < 0.02: #너무 작으면 버려 즉 정지해있으면 버려 근데 이건 내 배그 파일이 문제인 듯
                #     continue
                #만약 둘 박스의 둘 사이의 거리가 일정 이하가 된다고 한다면
                object_id = object_ids[row]
                prev_x = self.prev_objects[object_id].x_est[0]
                prev_y = self.prev_objects[object_id].y_est[0]
                # print(bboxes[col].predict_x[0], bboxes[col].predict_x[1])
                self.prev_objects[object_id].predict()
                self.prev_objects[object_id].update(bboxes[col].bbox[0], bboxes[col].bbox[1])
                self.prev_objects[object_id].bbox = bboxes[col].bbox
                self.prev_objects[object_id].bbox[0] = self.prev_objects[object_id].x_est[0]
                self.prev_objects[object_id].bbox[1] = self.prev_objects[object_id].y_est[0]
                new_x = self.prev_objects[object_id].x_est[0]
                new_y = self.prev_objects[object_id].y_est[0]
                mean_velo_x = velocity(self.prev_objects[object_id].velo_x, np.round((new_x - prev_x), 2))
                mean_velo_y = velocity(self.prev_objects[object_id].velo_y, np.round((new_y - prev_y), 2))
                self.prev_objects[object_id].bbox[6] = mean_velo_x
                self.prev_objects[object_id].bbox[7] = mean_velo_y
                print(np.round((mean_velo_x * 72),2))
                # print(mean_velo * 20 * 3.6)
                # object_id = row #물체의 id는 그 전에 저징이 되어있던 row값이 되는거고
                # self.prev_objects[object_id].P_k = P_k
                self.disappeared[object_id] = 0 # 잡히면 그 해당하는 물체는 count를 다시 0으로 가져감. 잡힌거니까 그 key에 해당하는 값은 0임. 이게 5를 넘으면 여기서 없애버릴것임
                # self.appeared[object_id] = 0 # 하나가 추가가 되었어
                used_rows.add(row)
                used_cols.add(col)


            unused_rows = set(range(cost_matrix.shape[0])) - used_rows # 새로 들어오지 않은 matrix들을 뽑아내면 즉 없었는데 생긴것들,
            unused_cols = set(range(cost_matrix.shape[1])) - used_cols # 있었는데 사라진 것들

            #기존 객체 삭제
            #여기서 기존의 객체가 사라져도 predict로 유지하도록 하는 코드를 추가해야함.
            for row in unused_rows: # 이전에는 있었는데 지금은 사라진 객체들에 대해서 말을 하는 것이고
                object_id = object_ids[row]
                self.disappeared[object_id] += 1 
                x_pred,y_pred = self.prev_objects[object_id].predict()
                self.prev_objects[object_id].x_est = x_pred
                self.prev_objects[object_id].y_est = y_pred
                
                self.prev_objects[object_id].bbox[0] = x_pred[0]
                self.prev_objects[object_id].bbox[1] = y_pred[0]
                if self.disappeared[object_id] > self.remove_count: 
                    self.remove(object_id)

            #새로운 객체 추가
            for col in unused_cols: 
                tmp_count = self.next_object_id
                tmp_count +=1
                self.check(tmp_count)
                # self.appeared[tmp_count] +=1
                # set boundary for large noise

                if self.appeared[tmp_count] > 20: 
                    
                    bboxes[col].set_initial_value(bboxes[col].bbox[0], bboxes[col].bbox[1])

                    #adding new object name traking it contains initial value, other infos for making marker
                    # tracking = KalmanFilter(0.1,0.5,0.5,1,0.1,0.1,  bboxes[col].bbox)
                    # tracking.set_initial_value([np.round(bboxes[col].bbox[0],2), np.round(bboxes[col].bbox[1],2)])
                    # P_k_minus_1 = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
                    # self.kf_list.append(kf)
                    self.push_back(bboxes[col]) 
            
            # for col in unused_cols: #원래는 없었는데 지금은 생긴 객체들에 대해서 말을 하는거임.
            #     if self.size(bboxes[col]) > 4: #지금은 size를 기반으로 객체를 추가를 하도록 하는데 앞으로는 센서 퓨전을 통해서 차량이나 사람 등 유의미한 객체에 대해서만 tracking을 하도록 하겠음.
            #     # tmp_count = self.next_object_id #얜 아직 +1이 되지 않은 상태
            #     # tmp_count +=1
            #     # self.check(tmp_count) #이번에 새로 생겼다면 그에 해당하는 box를 추가를 시켜주기.
            #     # self.appeared[tmp_count] +=1
            #     # if self.appeared[tmp_count] > 5:
            #         self.push_back(bboxes[col])
            self.tracker_pub.publish(self.publish_markers(self.prev_objects))
            self.text_pub.publish(self.publish_text(self.prev_objects))
            self.footprint_pub.publish(self.publish_future_points(self.prev_objects))
            self.dong_jang_feel_pub.publish(self.dong_jang_feel(self.prev_objects))
            end = time.time()
        else:
            print("no signal yet")    
                # print(end-first)
    def publish_future_points(self, objects):
        point_array = MarkerArray()
        for object_id in objects.keys():
            bbox = objects[object_id]
            marker = Marker()
            # 현재 위치도 추가..
            for i in range(5):
                xy = bbox.future_point()
                point = Point()
                if i<5:
                    point.x = xy[i][0]
                    point.y = xy[i][1]
                    point.z = 0.5
                    marker.points.append(point)
                # else:
                #     point.x = bbox.collision_pos_x
                #     point.y = bbox.collision_pos_y
                #     point.z = 0.5
                #     marker.points.append(point)
                
            # [[x1, y1, vx1, vx2], [x2, y2, vx2, vy2]]


            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "points_and_lines"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.r = 0.0
            marker.id = object_id
            marker.lifetime = rospy.Duration.from_sec(0.1)
            point_array.markers.append(marker)
        return point_array
    

    def dong_jang_feel(self, objects):
        object_info = PoseArray()
        object_info.header.frame_id = "base_link"
        object_info.header.stamp = rospy.Time.now()
        for object_id in objects.keys():
            bbox = objects[object_id]
            position = Pose()
            position.position.x = bbox.bbox[0]
            position.position.y = bbox.bbox[1]
            position.orientation.x = bbox.bbox[6]
            position.orientation.y = bbox.bbox[7]
            object_info.poses.append(position)
        return object_info


    def publish_text(self, objects):
        text_array  = MarkerArray()
        for object_id in objects.keys():
            bbox = objects[object_id]
            center_x = bbox.bbox[0]
            center_y = bbox.bbox[1]
            center_z = bbox.bbox[2]
            text = Marker()           
            text.id = object_id
            text.header.frame_id = "base_link"
            text.type = Marker.TEXT_VIEW_FACING
            text.text = "object_id : {0} \n velocity : {1}".format(object_id, bbox.bbox[6])
            text.action = Marker.ADD 
            text.pose.position.x = center_x
            text.pose.position.y = center_y
            text.pose.position.z = center_z + 0.5
            text.scale.z = 2
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 0.0
            text.color.b = 0.0
            text.lifetime = rospy.Duration.from_sec(0.1)
            text_array.markers.append(text)
        return text_array


    def publish_markers(self, objects):
        marker_array = MarkerArray()
        for object_id in objects.keys():
            bbox = objects[object_id]
            center_x = bbox.bbox[0]
            center_y = bbox.bbox[1]
            scale_x = bbox.bbox[3]
            scale_y = bbox.bbox[4]
            scale_z = bbox.bbox[5]
            ori_z = bbox.bbox[8]
            ori_w = bbox.bbox[9]
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.id = object_id
            marker.header.frame_id = 'base_link'
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0
            marker.pose.orientation.z = ori_z
            marker.pose.orientation.w = ori_w
            marker.scale.x = scale_x
            marker.scale.y = scale_y
            marker.scale.z = scale_z
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            marker.lifetime = rospy.Duration(0.1)
        return marker_array



if __name__=="__main__":
    rospy.init_node('tracker')
    tracker = ObjectTracker()
    # marker_sub = message_filters.Subscriber("/markers", MarkerArray)
    # detection_sub = message_filters.Subscirber("/jaejun/bbox_info", Detection2DArray)
    # ats = ApproximateTimeSynchronizer([marker_sub], queue_size=10, slop = 1.5)
    # ats.registerCallback(tracker.main)

    print("running")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tracker.main()
        rate.sleep()        
