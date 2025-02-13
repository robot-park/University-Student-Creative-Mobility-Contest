#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

i=0
def image_callback(msg):
    global i
    try:
        # cv_bridge를 사용하여 Image 메시지를 OpenCV 이미지로 변환합니다.
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "mono8")  # "mono8" indicates a grayscale image
        cv2.imshow("bev", cv_image)
        # 여기에 이미지 처리 및 기록 로직을 추가합니다.
        # 예를 들어, 이미지 파일로 저장하려면:
        output_folder = '/home/vision/lidar_ws/src/jaejun/src/image'
        filename = os.path.join(output_folder, "image"+str(i)+".png")
        cv2.imwrite(filename, cv_image)
        i+=1
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr("Error processing the image: %s", str(e))

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/bev_image', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
