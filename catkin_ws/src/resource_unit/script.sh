#!/bin/sh
cd ../..
pip install matplotlib
catkin_make
bash devel/setup.bash 
