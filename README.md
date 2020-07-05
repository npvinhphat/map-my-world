# Map My World

This is the fourth project for Udacity's [Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209), namely *Map My World*.

## How it works

The main package for this project is **my_robot**, which houses the robot, including the camera module and the lidar module associated, as well as the world which the robot will be instantitated in.

This robot uses [RTAP-MAP](http://wiki.ros.org/rtabmap_ros) ROS package to perform SLAM. RTAP-Map is a RGB-D SLAM approach with real-time constraint, taking input as image an generate 3D point cloud of the environment, as well as the 2D occupancy grid map for navigation.

Most of the configuration is stored in `mapping.launch` file, which indicates parameters such as how to extract feature from the image (the default is using [SURF](https://en.wikipedia.org/wiki/Speeded_up_robust_features)). The launch file also bundles with `rtabmapviz` to show the mapping and localization in real-time.

This repos also includes a package named **teleop_twist_keyboard** as a simple mean to control the robot. The documentation can be seen [here](http://wiki.ros.org/teleop_twist_keyboard).

## Installation

Make sure that you have [installed Catkin](http://www.ros.org/wiki/catkin#Installing_catkin) and sourced your environment. More info about creating your catkin workspace [here](http://www.ros.org/wiki/catkin#Installing_catkin).

Use the following commands to build the modules:

```
cd ~/catkin_ws/src
git pull git@github.com:npvinhphat/map-my-world.git .
cd ..
catkin_make
```

## Usage

Run the following commands to open Gazebo and Rviz, and launch the world:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch
```

In order to run the mapping node for the robot to perform SLAM, **open a new terminal** and do the following:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_robot mapping.launch
```

This would initializes the SLAM's rtab node. There will be nothing at first, and you have to move the robot around to close the loop. This can be done by **open a new terminal** and do the below, followed by the instruction from the terminal:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

The recommendation is to move the robot around in close loop by three times. After then, your mapping performance will be stored in `~/.ros/rtabmap.db` file. To open the file, uses the following command:

```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

The interface should looks like below, showing the the entire SLAM performance as well as how many loop closure has been done. The latest perfomance is shared [here](https://drive.google.com/file/d/1PTa_wK8Um6BZIF-nQ8u23s9JtP-_U4hJ/view?usp=sharing) on Google Drive.

![](https://user-images.githubusercontent.com/10416670/86534603-7caa2d00-bf14-11ea-8d81-300818d6c2ec.png)
