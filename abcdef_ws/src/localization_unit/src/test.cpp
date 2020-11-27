#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"

int main(){
    rclcpp::Clock::SharedPtr clock;
    tf2_ros::Buffer buffer = tf2_ros::Buffer(clock,tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));

}