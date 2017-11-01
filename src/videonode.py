import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

fourcc_codec = cv2.VideoWriter_fourcc(*"DIVX")
video_writer1 = cv2.VideoWriter("output4.avi", fourcc_codec, 20.0, (640, 360), isColor=True)
# video_writer2 = cv2.VideoWriter("output2.avi", fourcc_codec, 20.0, (640, 360), isColor=True)

cvbridge= CvBridge()
def write_image_callback1(data):
    cv_img = cvbridge.imgmsg_to_cv2(data)
    video_writer1.write(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

# def write_image_callback2(data):
#     cv_img = cvbridge.imgmsg_to_cv2(data)
#     video_writer2.write(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

rospy.init_node("video_writer")
sub1 = rospy.Subscriber("/ardrone/front/image_raw", Image, write_image_callback1)
# sub2 = rospy.Subscriber("/ardrone2/ardrone/front/image_raw", Image, write_image_callback2)
rospy.spin()