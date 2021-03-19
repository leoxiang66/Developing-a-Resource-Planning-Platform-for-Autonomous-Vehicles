# Developers

This project is done by Tao Xiang and Mehdi Kallel. Tao has done kalmen filter in localization unit and object detection in perception unit. Mehdi has done RF2O in localization unit and object aggregator and object classification in perception unit. We finish the rest part together. Please take into consideration that we also helped each other for our own parts.



# ReadMe all packgages

First of all, install the Autoware AI following the steps in https://autoware.readthedocs.io/en/feature-documentation_rtd/UsersGuide/Installation.html#spec-recommendation
Then, follow the steps below to compile and run the packages.

1. **simulator**
    ```
    cd lgsvl.../lgsvl...
    ./simulator
    ```

  simulations > 2nd one > play > play

2. **runtime manager**  
    ```
    cd docker/generic
    ./run.sh
    cd shared_dir/project-gitlab/pu_ws
    bash script.sh
    source devel/setup.bash
    catkin_make
    roslaunch runtime_manager runtime_manager.launch
    ```
    `Ref`: autoware/shared_dir/autoware-data/BorregasAve/my_launch/my_xxx.launch
    then load
3. **object detection**
    ```
    cd docker/generic
    ./run.sh
    cd shared_dir/project-gitlab/pu_ws
    bash script.sh
    source devel/setup.bash
    catkin_make
    roslaunch perception_unit voxel_grid_filter.launch
    ```
    
    in `runtime manager`: Computing > lidar_euclidean_cluster_detect > app > voxel_filter_z/output 
    - clustering distance: 2.25
    - clip_min_height: -1.5
    - remove_points_upto: 2.15
    
    check `lidar_euclidean_cluster_detect` and `lidar_shape_estimation`
    
    sensing > calibration_publisher>Ref: autoware/shared_dir/autoware-data/BorregasAve/data/calibration/camera_lidar_3d/pirus/2nd one > Ok 
    
    
    
4. **object classification**
    ```
    cd docker/generic
    ./run.sh
    cd shared_dir/project-gitlab/pu_ws
    bash script.sh
    source devel/setup.bash
    catkin_make
    rosrun perception_unit obj_detection_node
    ```
    
5. **localization**
    ```
    cd docker/generic
    ./run.sh
    cd shared_dir/project-gitlab/catkin_ws
    bash script_all.sh
    ```
    `Rviz` > 2D estimation > ... > fixed frames: base_link
    
    ```
    source devel/setup.bash
    catkin_make
    roslaunch lu lu_unit.launch
    ```

6. **resource planner**
    ```
    cd docker/generic
    ./run.sh
    cd shared_dir/project-gitlab/pu_ws/
    bash script.sh
    ```
    
    in the `simulator`: Enviroment > traffic + pedestrains
    
    ```
    source devel/setup.bash
    catkin_make
    roslaunch resource_unit resource_monitor.launch
    ```
    
    frequency: how often a msg is published
    latency: how long it takes for the msg to arrive at the latency node

# Demo

[bilibili](https://www.bilibili.com/video/BV1SU4y1H7ZE)




