#!/usr/bin/env python
import rospy
from perception_unit.msg import Latency
from perception_unit.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry

# Class to watch the latency of a given topic
class LatencyMonitor():
    def __init__(self, topic_name, ros_msg_type):
        rospy.logwarn("initializing latency_monitor")
        self.Latency_mag = Latency()
        rospy.Subscriber(topic_name, ros_msg_type, self.callback_lm)
        self.pub_latency_monitor = rospy.Publisher('/latency_monitor_' + topic_name, Latency, queue_size=10)

    def callback_lm(self, msg):
        # Calculate the latency using the timestamp of the published msg
        self.Latency_mag.header.stamp = rospy.Time.now()
        try:
            time_before = msg.markers[0].header.stamp
        except:
            time_before = msg.header.stamp

        # The latency should be the difference from received time and published time
        delta_t =  rospy.Time.now() - time_before
        self.Latency_mag.latency = delta_t.to_sec()
        # Publish the result
        self.pub_latency_monitor.publish(self.Latency_mag)

if __name__ == "__main__":
    try:
        rospy.init_node('latency_monitor_node')
        ros_msg_type = MarkerArray
        topic_name = "/detection/shape_estimation/objects_markers"
        # Watch the following topics
        class_LM1 = LatencyMonitor("detection/shape_estimation/objects_markers", MarkerArray)
        class_LM2 = LatencyMonitor("detection/lidar_detector/objects_markers", MarkerArray)
        class_LM3 = LatencyMonitor("obj_detection/detected_objects", DetectedObjectArray)
        class_LM4 = LatencyMonitor("ekf/odometry", Odometry)
        class_LM5 = LatencyMonitor("rf2o/odometry", Odometry)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
