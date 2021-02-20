#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# Topic subscribers declaration
ekf_pub = rospy.Publisher('ekf/odometry', Odometry, queue_size=10)
rf2o_pub = rospy.Publisher('rf2o/odometry', Odometry, queue_size=10)

# Timing topic callback
def TimeCallback(msg):
    rospy.loginfo("[LOCALIZATION]: Received timing msg")
    rospy.logerr(msg.data)

# Ekf odometry topic callback
def EkfCallback(data):
    ekf_pub.publish(data)
    rospy.loginfo("kalmann filter{Pose(%f,%f,%f),Velocity(%f,%f,%f)}",data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,
    data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z)

# Rf2o odometry topic callback
def Rf2oCallback(data):
    rf2o_pub.publish(data)
    rospy.loginfo("RF2O{Pose(%f,%f,%f),Velocity(%f,%f,%f)}",data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,
    data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z)

# Ros loop
def main():
    rospy.init_node('lu_unit', anonymous=True)
    rospy.Subscriber("/odometry/filtered_odom", Odometry, EkfCallback)
    rospy.Subscriber("/odom_rf2o", Odometry, Rf2oCallback)
    rospy.Subscriber("/localization_timing", String, TimeCallback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


