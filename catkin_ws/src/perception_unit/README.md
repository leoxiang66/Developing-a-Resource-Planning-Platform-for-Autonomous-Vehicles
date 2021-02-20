# perception_unit

Dependecies:
- ros melodic
- image proc package
- image tranportation
- pcl_ros
- Autoware.ai

To install the dependecies run:
`sh scrip.sh`

Usage
- Download the packages in the dir catkin_ws/src that is inside the shared_dir
- Enter in the docker conteiner for autoware ai
- Enter in shared_dir/catkin_ws
- Run: `catkin_make`
- To run the perception unit run the following command:

`roslaunch perception_unit voxel_grind_filter.launch`

- Using the runtime manager, run the euclidian cluster node and the shape estimation node in sensing tab.
Change the input topic of the euclidian cluster for `voxel_filter_z/output`
