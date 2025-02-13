from ukf import Object
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

"""

무향 칼만 필터의 동작 과정

추정치 : 예측과 update를 모두 거친 값 / 측정치 : 센서로부터 받은 데이터
1. t시점에 이전에 예측한 내 위치 추정치를 가지고 있음
2. 이 추정치 주변에 시그마 포인트들을 배치
3. t+1 시점으로 이동하기 위해 시그마 포인트들을 시스템 모델에 집어넣어 전진시킴
    ->각각 시그마 포인트들에 대해서 시그마 포인트들의 예측 위치가 나오게됨
4. 예측된 위치들을 평균을 내어 t+1시점에서 예측값을 얻음
5. t+1시점에서 새로운 측정값이 들어오게 됨
6. 이 새로운 측정값과 우리가 예측한 값 사이의 차이를 줄여주기 위해 업데이트를 함
    -> 추정치를 수정하는거지(이 때 공분산을 사용하는 것이지)


여기서 공분산의 개념을 좀 자세히 설명해보면
먼저 우리는 두 개의 공분산을 설정하는데 
하나는 프로세스 노이즈->우리의 예측이 정확하지 않을 것이라는 것을 전제로 끼워넣은 노이즈
다른건 측정 노이즈->센서 자체에서 오차가 어느 정도 포함된 값이 나올거라는 걸 전제로 끼워넣은 노이즈
업데이트 단계에서 이제 오차 공분산을 계산하는데 이 오차 공분산은 잔차(측정값-이전시점에서 구한 추정값)를
사용하여 오차 공분산을 업데이트 한다 이때 칼만이득이라는 것을 산출한다.
    

"""

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
# class Object:
#     def __init__(self, idx, bbox):
#         self.idx = idx
#         self.bbox = bbox
#         self.age = 0
#         self.P_k=0

class ObjectTracker:
    def __init__(self,  remove_count=15, distance_threshold=1, appear_count=10):
        self.next_object_id = 0
        self.prev_objects = {}
        self.disappeared = {}
        self.appeared = {}
        self.remove_count = remove_count
        self.distance_threshold = distance_threshold
        self.appear_count = appear_count
        rospy.Subscriber('/markers', MarkerArray, self.callback)
        self.tracker_pub = rospy.Publisher("/tracked_marker", MarkerArray, queue_size=10)    
        self.footprint_pub = rospy.Publisher("/foot_print", MarkerArray, queue_size=10) #�ㅻ뒗 湲몄뿉 珥앹킑珥앹킑 �④린怨� �띠뿀�쇰굹 ros 硫붿떆吏��� �섎굹�� 諛뽰뿉 �� �� �놁쓬. �곕씪�� �щ윭媛쒕� �댁빞 珥앹킑珥� �⑥쓣�먮뜲 援녹씠?
        self.text_pub = rospy.Publisher("/text", MarkerArray, queue_size=10)
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
        # �대� �덈뒗 嫄곕옉 �꾩떆濡� �レ옄 �섎젮�볦� 嫄곕옉
        # 紐� �꾨젅�� �댁긽 �ㅼ뼱�ㅻ㈃ 洹� �� �밤뀋媛앹껜濡� 異붽�.
        if count != len(self.appeared):
            self.appeared[count] = 0
        else:
            self.appeared[count] +=1

    def push_back(self, object): #�쇰떒 異붽��� �덈뒗�� �ㅼ쓬 �꾨젅�꾩뿉�쒕룄 異붽�媛� �섏뼱�� �� 媛앹껜瑜� �좎��� 寃� �꾨깘 留욎�. 
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

    def distance(self, prev_object, bbox): #媛앹껜�ъ씠�� 嫄곕━瑜� 湲곕컲�쇰줈 �닿쾶 媛숈� 臾쇱껜�몄� tracking�� �� �덉젙��. -> 異뷀썑�� 媛숈� id�� 媛앹껜�� 媛믪씠 �덈Т ���� 寃쎌슦媛� 諛쒖깮�섎㈃ �대� 蹂댁젙�섎룄濡� �섎뒗 肄붾뱶�� 異붽��� �덉젙
        prev_center_x, prev_center_y = prev_object.ukf_x.x[0], prev_object.ukf_y.x[0] #�댁쟾 �꾨젅�꾩뿉�쒖쓽 媛앹껜�� 媛��대뜲 醫뚰몴
        new_center_x, new_center_y = bbox.bbox[0], bbox.bbox[1] #�꾩옱 �꾨젅�꾩뿉�� 媛앹껜�� 媛��대뜲 醫뚮즺
        distance = math.sqrt((prev_center_x - new_center_x)**2 + (prev_center_y - new_center_y)**2) #嫄곕━怨꾩궛
        return distance
    
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
            # print("tracking")
            print("number of object :", self.next_object_id)
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

            object_ids = list(self.prev_objects.keys())
            cost_matrix = np.zeros((len(self.prev_objects), len(bboxes)))

            for i in range(len(object_ids)):
                for j in range(len(bboxes)):
                    cost_matrix[i, j] = self.cost(self.prev_objects[object_ids[i]], bboxes[j]) 
            
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            
            used_rows = set()
            used_cols = set()

            for row, col in zip(row_ind, col_ind): 
                if row in used_rows or col in used_cols: 
                    continue

                if cost_matrix[row, col] > 1.0 : 
                    continue
                

                object_id = object_ids[row]
                z_t1 = [bboxes[col].bbox[0], bboxes[col].bbox[1]]
                bbox_t1  = bboxes[col].bbox
                self.prev_objects[object_id].Predict()
                
                self.prev_objects[object_id].Update(z_t1, bbox_t1)
                
                updated_x = self.prev_objects[object_id].ukf_x.x[0]
                updated_y = self.prev_objects[object_id].ukf_y.x[0]
                

                self.disappeared[object_id] = 0 
                used_rows.add(row)
                used_cols.add(col)


            unused_rows = set(range(cost_matrix.shape[0])) - used_rows 
            unused_cols = set(range(cost_matrix.shape[1])) - used_cols

            #기존에 있던 값에서 만약에 사라졌다면
            for row in unused_rows: # 원래 측정이 되어있던 공분산을 이용해서 predict를 계속 수행하도록 하였음
                object_id = object_ids[row]
                self.disappeared[object_id] += 1 
                self.prev_objects[object_id].Predict()
                x_pred = self.prev_objects[object_id].ukf_x.x[0]
                y_pred = self.prev_objects[object_id].ukf_y.x[0]

                self.prev_objects[object_id].bbox[0] = x_pred
                self.prev_objects[object_id].bbox[1] = y_pred
                if self.disappeared[object_id] > self.remove_count: 
                    self.remove(object_id)

            # 새로운 값이 생성 col이 새로 들어온 값이고 row가 기존에 있던 값임
            for col in unused_cols: 
                tmp_count = self.next_object_id
                tmp_count +=1
                self.check(tmp_count)


                if self.appeared[tmp_count] > 10: 
                    bboxes[col].set_initial_value([bboxes[col].bbox[0], bboxes[col].bbox[1]])
                    self.push_back(bboxes[col]) 
            

            self.tracker_pub.publish(self.publish_markers(self.prev_objects))
            self.text_pub.publish(self.publish_text(self.prev_objects))
            # self.footprint_pub.publish(self.publish_future_points(self.prev_objects))
            end = time.time()
        else:
            print("no signal yet")    
                # print(end-first)
    def publish_future_points(self, objects):
        point_array = MarkerArray()
        for object_id in objects.keys():
            bbox = objects[object_id]
            marker = Marker()
            for i in range(6):
                xy = bbox.future_point()
                point = Point()
                if i<5:
                    point.x = xy[i][0]
                    point.y = xy[i][1]
                    point.z = 0.5
                    marker.points.append(point)
                else:
                    point.x = 0
                    point.y = bbox.collision_pos_y
                    point.z = 0.5
                    marker.points.append(point)
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "points_and_lines"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.id = object_id
            marker.lifetime = rospy.Duration.from_sec(0.1)
            point_array.markers.append(marker)
        return point_array

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
            text.text = "object_id : {0}, velocity : {1}".format(object_id, bbox.bbox[6])
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
            marker.color.a = 0.5
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