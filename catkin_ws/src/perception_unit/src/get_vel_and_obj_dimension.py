#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from perception_unit.msg import Vel


class GetVel():
    def __init__(self):
        rospy.logwarn("initializing get_vel_and_cube")
        rospy.Subscriber("/detection/shape_estimation/objects_markers", MarkerArray, self.callback)
        self.pub_vel = rospy.Publisher('/vel_and_obj_dimension', Vel, queue_size=10)
        self.vel = Vel()
        self.marker_old = {}

    def callback(self, marker_array):
        for marker in marker_array.markers:
            if marker.ns == "/detection/shape_estimation/box_markers":
                self.cal_vel(marker, self.marker_old)
                self.marker_old[marker.id] = [marker.header.stamp, marker.pose.position.x,
                    marker.pose.position.y, self.vel.vel_x, self.vel.vel_y]

    def cal_vel(self, marker, marker_old):
        try:
        #     rospy.logwarn(marker_old[marker.id])
        # except:
        #     pass
            delta_t = marker.header.stamp.to_sec() - marker_old[marker.id][0].to_sec()
            # rospy.loginfo(delta_t)

            delta_x = marker.pose.position.x - marker_old[marker.id][1]
            delta_y = marker.pose.position.y - marker_old[marker.id][2]

            self.vel.header.stamp = marker.header.stamp
            self.vel.id = marker.id
            self.vel.vel_x = delta_x/delta_t 
            self.vel.vel_y = delta_y/delta_t

            if abs(self.vel.vel_x - marker_old[marker.id][3]) > 30.0:
                self.vel.vel_x = 0
                self.vel.vel_y = 0
            if abs(self.vel.vel_y - marker_old[marker.id][4]) > 30.0:
                self.vel.vel_x = 0
                self.vel.vel_y = 0

            self.vel.pose.position.x = marker.pose.position.x
            self.vel.pose.position.y = marker.pose.position.y

            self.vel.dimension.x = marker.scale.x
            self.vel.dimension.y = marker.scale.y
            self.vel.dimension.z = marker.scale.z

            
            self.pub_vel.publish(self.vel)
        except:
            rospy.logwarn("Did not find id: " + str(marker.id))
    
if __name__ == "__main__":
    try:
        rospy.init_node('get_vel_and_cube')
        class_sp = GetVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass