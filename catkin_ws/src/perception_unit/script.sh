#!/bin/sh
cd ../..
catkin_make
pip install matplotlib
sudo apt update
sudo apt install ros-melodic-pcl-ros
sudo apt install ros-melodic-image-proc 
sudo apt install ros-melodic-depth-image-proc 
sudo apt install ros-melodic-image-transport
source devel/setup.bash 