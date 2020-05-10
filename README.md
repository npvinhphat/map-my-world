# Go Chase It

This is the second project for Udacity's [Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209), namely *Go Chase It!*.

## How it works

The project is split into two packages: **my_robot** and **ball_chaser**.

**my_robot** houses the robot, including the camera module and the lidar module associated, as well as the world which the robot will be instantitated in. There are two topics to take note here:
* `/cmd/vel`: control the movement of the robot
* `/camera/rgb/image_raw`: the raw image from robot's camera

**ball_chaser** houses the logic the chasing the white ball, including two nodes:
* **drive_bot**: houses a service `/ball_chaser/command_robot` with `ball_chaser/DriveToTarget` input/output to publish the *linear_x* ang *angular_z* drive command to `/cmd/vel`
* **process_image**: subsribe to `/camera/rgb/image_raw`, do some simple logic to determine the white ball position, and calls `/ball_chaser/command_robot` to drive the robot

Simple illustration below:

![go-chase-it-architecture](https://user-images.githubusercontent.com/10416670/81492756-6e091600-92d5-11ea-94b6-cbe4c25ecd4b.png)

## Installation

Make sure that you have [installed Catkin](http://www.ros.org/wiki/catkin#Installing_catkin) and sourced your environment. More info about creating your catkin workspace [here](http://www.ros.org/wiki/catkin#Installing_catkin).

Use the following commands to build the modules:

```
cd ~/catkin_ws/src
git pull git@github.com:npvinhphat/go-chase-it.git .
cd ..
catkin_make
```

## Usage

Run the following commands to open Gazebo and Rviz:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch
```

Then your world should look the same as below, with a white ball and a robot:

![go-chase-it-gazebo](https://user-images.githubusercontent.com/10416670/81492119-58ddb880-92d0-11ea-94be-1991c388adad.png)

In order to run the node for the robot to chase the white ball, **open a new terminal** and do the following:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

Then in Gazebo, drag the white ball in front of the robot and see the result.
