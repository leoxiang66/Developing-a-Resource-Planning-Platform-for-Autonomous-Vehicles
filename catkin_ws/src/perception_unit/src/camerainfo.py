#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, CompressedImage
import message_filters
from cv_bridge import CvBridge


msg = CameraInfo()
def callback(ros_data):
    global image_pub
    global msg
    # bridge = CvBridge()
    np_arr = np.fromstring(ros_data.data, np.uint8)
    bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # bgr_image = np.fromstring(ros_data.data, np.uint8)
    b_img,g_img,r_img = cv2.split(bgr_image)
    ros_data.data = np.array(cv2.imencode('.jpg', b_img)[1]).tostring()
    stamp = rospy.get_rostime()
    ros_data.header.stamp = stamp
    msg.header.stamp = stamp
    pub_dep.publish(msg)
    pub_seg.publish(msg)
    image_pub.publish(ros_data)

def talker():
    global image_pub
    global pub_seg
    global pub_dep
    global msg
    rospy.init_node('camera_info_segmented', anonymous=False)
    pub_seg = rospy.Publisher('/simulator/camera_info', CameraInfo, queue_size=1)
    pub_dep = rospy.Publisher('/simulator/depth_forward/camera_info', CameraInfo, queue_size=1)
    image_pub = rospy.Publisher("/simulator/depth_forward/compressed", CompressedImage, queue_size=1)
    # self.bridge = CvBridge()

    # subscribed Topic
    subscriber = rospy.Subscriber("/simulator/depth_camera/compressed", CompressedImage, callback,  queue_size = 1)
    rate = rospy.Rate(13) # 10hz
    width = 1080
    height = 1080
    fov = 50
    fx = height / (2 * math.tan(fov * math.pi / 360))    # ImageSizeX /(2 * tan(CameraFOV * pi / 360))
    fy = width / (2 * math.tan(fov * math.pi / 360)) # ImageSizeX /(2 * tan(CameraFOV * pi / 360))
    fy = 0.5*(height/(np.tan(np.deg2rad(fov/2))))
    fx = 0.5*(height/(np.tan(np.deg2rad(fov/2))))
    msg = CameraInfo()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = 'velodyne'
    msg.distortion_model = 'plumb_bob'
    msg.D = [0,0, 0,0,0]

    msg.width = width
    msg.height = height
    msg.K[0] = fx       # fx
    msg.K[1] = 0
    msg.K[2] = width/2. # cx
    msg.K[3] = 0
    msg.K[4] = fy       # fy
    msg.K[5] = height/2. # cy
    msg.K[6] = 0
    msg.K[7] = 0
    msg.K[8] = 1

    msg.R[0] = 1
    msg.R[4] = 1
    msg.R[8] = 1

    msg.P[0] = fx
    msg.P[2] = width/2.
    msg.P[5] = fy
    msg.P[6] = height/2.
    msg.P[10] = 1

    rospy.spin();
    # while not rospy.is_shutdown():
    #     # hello_str = "hello world %s" % rospy.get_time()
    #     # rospy.loginfo(hello_str)
    #     pub_dep.publish(msg)
    #     pub_seg.publish(msg)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass