#!/usr/bin/python2
import rospy
import geometry_msgs.msg
import tf
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from perception_unit.msg import Vel
from perception_unit.msg import DetectedObjectArray
from std_msgs.msg import String
import math
# import tf

STATIC_VEL_OBJ = 1

# Detection Objects Aggregator Class
# Subscribes velocity estimation and objects detected and perception timing errors
class DOAggregator():
    def __init__(self):
        rospy.Subscriber("/vel_and_obj_dimension", Vel, self.callback_velocity)
        rospy.Subscriber("/obj_detection/detected_objects", DetectedObjectArray, self.callback_types)
        rospy.Subscriber("/perception_timing", String, self.callback_timing)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # Callback for timing errors from monitor
    def callback_timing(self, msg):
        rospy.loginfo("[PERCEPTION]: Received timing msg")
        rospy.logerr(msg.data)

    # Callback for receiving velocity msg
    def callback_velocity(self, msg):
        # rospy.loginfo("Received dimmensions and velocities msg")
        # Logger the received msg
        flag = False
        pose_input = PoseStamped()
        pose_input.header = 'base_link'
        pose_input.pose = msg.pose

        # Transform the pose in car frame to the map frame so we can show the right direction
        try:
            transform = self.tf_buffer.lookup_transform('map',
                                       'base_link', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
            flag = True
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_input, transform)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            flag = False
            pose_transformed = msg.pose

        # Calculate the velocity from axis
        moving_vel = math.sqrt(msg.vel_x**2 + msg.vel_y**2)
        # Check if the object is a moving object
        if moving_vel > STATIC_VEL_OBJ:
            x = pose_transformed.pose.position.x
            y = pose_transformed.pose.position.y
            if x > 0:
                x_dir = 'north'
            else:
                x_dir = 'south'
            if y > 0:
                y_dir = 'west'
            else:
                y_dir = 'east'
            # Check if the object is bigger than 2 m each side then must be a moving car
            if msg.dimension.x > 2 or msg.dimension.y > 2:
                obj_type = 'car'
            else:
                obj_type = 'person'

            # Log the moving objects
            rospy.loginfo('There is a %s moving in direction %s %s with velocity %3f m/s' % (obj_type, x_dir, y_dir, moving_vel))

    # Logger the received msg
    def callback_types(self, msg):
        # rospy.loginfo("Received types msg")
        rospy.loginfo(msg)

# Main
if __name__ == "__main__":
    try:
        rospy.init_node('objects_detector_aggregator')
        doaggregator = DOAggregator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
