import rospy
from vision_msgs.msg import Detection2DArray

def callback(msg):
    print(msg.detections[0].results[0].score)



if __name__=="__main__":
    rospy.init_node("init")
    rospy.Subscriber("/sign_bbox", Detection2DArray, callback, queue_size=10)
    rospy.spin()