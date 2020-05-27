# Where Am I

This is the third project for Udacity's [Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209), namely *Where Am I*.

## How it works

The main package for this project is **my_robot**, which houses the robot, including the camera module and the lidar module associated, as well as the world which the robot will be instantitated in.

This robot uses [AMCL](http://wiki.ros.org/amcl) package to perform localization. Most parameters can be configured in `amcl.launch` file, under `/my_robot/launch/` folder.

To visualize on Rviz, there is also a generated map (under `/my_robot/maps/` folder) using Udacity's provided package [pgm_map_creator](https://github.com/udacity/pgm_map_creator).

This repos also includes a package named **teleop_twist_keyboard** as a simple mean to control the robot. The documentation can be seen [here](http://wiki.ros.org/teleop_twist_keyboard).

## Installation

Make sure that you have [installed Catkin](http://www.ros.org/wiki/catkin#Installing_catkin) and sourced your environment. More info about creating your catkin workspace [here](http://www.ros.org/wiki/catkin#Installing_catkin).

Use the following commands to build the modules:

```
cd ~/catkin_ws/src
git pull git@github.com:npvinhphat/where-am-i.git .
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

In order to run the amcl node for the robot to perform localization, **open a new terminal** and do the following:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_robot amcl.launch
```

Then in Rviz, load the config `config.rviz` to show the localization. The red arrow is the particles in AMCL algorithm. Your Rviz should looks like below:

![](https://user-images.githubusercontent.com/10416670/83043624-0cd79580-a07e-11ea-943d-89cd28bd84bb.png)

Finally, use the teleop to move the robot around and test your localization performance, by **open a new terminal** and do the below, followed by the instruction from the terminal:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

By moving around, the particles should converges, showing that the robot is being localized:

![](https://user-images.githubusercontent.com/10416670/83043607-09440e80-a07e-11ea-8edb-fa8a6bd410ae.png)