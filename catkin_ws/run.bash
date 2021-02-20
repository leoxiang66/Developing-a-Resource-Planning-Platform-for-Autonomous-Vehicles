#!/bin/bash
chmod +x src/localization_unit/lu/src/*.py
chmod +x src/localization_unit/nmea_navsat_driver/scripts/*
chmod +x src/perception_unit/src/*.py
chmod +x src/resource_unit/src/*.py
source devel/setup.bash
roslaunch perception_unit run.launch