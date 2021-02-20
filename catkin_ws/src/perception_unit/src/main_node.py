#!/usr/bin/env python
import rospy
import rospkg


if __name__ == '__main__':
    rospy.init_node('perception_unit')
    rospy.logwarn("Starting perception unit node)

    try:
        # start class

        rospy.spin()

    except rospy.ROSInterruptException:
        pass