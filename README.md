# Basalt VIO ROS2 wrapper

This is a ROS2 wrapper around
the [Basalt VIO library](https://gitlab.com/VladyslavUsenko/basalt).


Note that the wrapper is split into two parts: frontend and
backend. The frontend performs the image feature extraction, the
backend runs the graph optimizer etc. The front end publishes messages
that contain the extracted features. These messages can be recorded in
a rosbag and played back, such that there is no need to store the
images to test the backend.

For efficiency reasons, front- and backend are fused together into a
single exectuable as well, avoiding the overhead of serializing and
de-serializing ROS messages from the Basalt data structures.

# Supported systems

Only tested on Ubuntu 18.04LTS with ROS2 Eloquent

# Building

At the root of the workspace, do:

    cd path_to_top_of_your_workspace
	colcon build --symlink-install --cmake-args	-DCMAKE_BUILD_TYPE=Release

# Testing

Overlay your workspace:

    cd path_to_top_of_your_workspace
	. install/setup.bash   # activate this workspace

Launch the unified front/backend as a node:

    ros2 launch basalt_ros2 basalt_vio.launch.py
	
Now you need supply data from the realsense T265.

# How to get the RealSense calibration

To get the calibration file, do this:

    ros2 run basalt_ros2 get_calibration  -o my_calib_file.json
        
If you want to overwrite the imu noise with your own idea of
covariance:

    ros2 run basalt_ros2 get_calibration  -o my_calib_file.json -a 0.5
