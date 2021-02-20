#!/bin/sh
cd ../..
sudo apt update
# pip install matplotlib numpy
sudo apt update
sudo apt install -y python-matplotlib python-numpy
sudo apt install -y ros-melodic-pcl-ros
sudo apt install -y ros-melodic-image-proc
sudo apt install -y ros-melodic-image-proc
sudo apt install -y python3-rospkg python3-numpy python-rospkg
sudo apt install -y ros-melodic-depth-image-proc 
sudo apt install -y ros-melodic-image-transport
catkin_make
bash devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
sudo apt-get install -y ros-melodic-pointcloud-to-laserscan
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
            
bash devel/setup.bash 
#pip3 install rospkg
pip3 install numpy