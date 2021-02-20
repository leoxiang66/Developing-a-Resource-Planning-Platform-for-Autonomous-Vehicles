# Developers

This project is done by Tao Xiang and Mehdi Kallel. Tao has done kalmen filter in localization unit and object detection in perception unit. Mehdi has done RF2O in localization unit and object aggregator and object classification in perception unit. We finish the rest part together. Please take into consideration that we also helped each other for our own parts.



# ReadMe all packgages

First of all, install the Autoware AI following the steps in https://autoware.readthedocs.io/en/feature-documentation_rtd/UsersGuide/Installation.html#spec-recommendation
Then, follow the steps below to compile and run the packages.

- Run the LGVSL Simulator. Inside its dir run:
	./simulator

- Choose Borregas Ave that uses Autoware. And play the simulation.

- Run the Autoware 1.14.0 container and enter into it:
	cd docker/generic
	./run.sh -t 1.14.0

- Run the runtime manager
	roslaunch runtime_manager runtime_manager.launch

- Set the simulation
	A few terminals will open, as well as a GUI for the runtime manager. In the runtime manager, click on the 'Quick Start' tab and load the following launch files from ~/shared_dir/autoware-data/BorregasAve/ by clicking "Ref" to the right of each text box.

- Set the lidar calibration in the Sensing tab. Click in Calibration Publisher and copy-paste the follwoing path to the calibrration file in the textbox and click OK
	/home/autoware/shared_dir/autoware-data/BorregasAve/data/calibration/camera_lidar_3d/prius/camera_lidar_sim_sf.yaml

- The vehicle may be mis-localized as the initial pose is important for NDT matching. To fix this, click "2D Pose Estimate" in Rviz, then click an approximate position for the vehicle on the map and drag in the direction it is facing before releasing the mouse button.

- Go to the Computing tab in runtime manager and click in the app link for lidar_euclidian_cluster_detect and change the input topic for
	voxel_filter_z/output and set the parameters like the following: 
		- clustering distance: 2.25
		- clip_min_height: -1.5
		- remove_points_upto: 2.15

- And run the nodes lidar_euclidian_cluster_detect and lidar_shape_estimation by clicking on their respective boxes

- Inside the docker install the python3-rospkg using pip
	pip3 install rospkg

- Update the apt repos
   sudo apt update

- Install the ros dependecies using apt
   sudo apt install -y python-matplotlib python-numpy ros-melodic-pcl-ros ros-melodic-image-proc ros-melodic-depth-image-proc ros-melodic-image-transport

- Go to the catkin_ws dir
	cd shared_dir/catkin_ws

- Run the dependencies script to install the remaining dependencies
   bash script_all.sh

- Compile the workspace
	catkin_make

- Setup the evironment
	source devel/setup.bash

- Run all packages
Using the `run.bash` script will bring up all nodes for all units and run them.
	bash run.bash