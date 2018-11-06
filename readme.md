## ROS tutorials basics

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Prerequisites (Installation)

ROS Installation:
The following code is run over ROS Kinetic on Ubuntu 16.04 LTS.

To install ROS Kinetic:
http://wiki.ros.org/kinetic/Installation

Installing catkin:
http://wiki.ros.org/catkin

Click on the Kinetic tab on the above link and proceed as instructed.

## Adding shortcut on bashrc
```
sudo nano ~/.bashrc
```
Navigate to the end of the file and add:

```
source ~/catkin_ws/devel/setup.bash
```

You dont have to keep sourcing the setup.bash file inside the devel folder inside
your ROS workspace everytime. Whenever a new tab or window of the terminal is opened,
bashrc is automatically run.

## Setting up ROS workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/.bashrc
cd src/
git clone -b Week10_HW --single-branch https://github.com/AdarshJS/beginner_tutorials.git
cd ..
catkin_make
source ~/.bashrc
```

## Running the code

Open a new terminal and start ROS MASTER

```
roscore
```

On a new tab:

```
rosrun beginner_tutorials talker
```

On another tab:
```
rosrun beginner_tutorials listener
```

The talker broadcasts a string (my name in this case).
The listener should print "I heard: <my name>"

## Running code using launch file
In a new terminal:
```
roslaunch beginner_tutorials basic.launch
```
This runs talker and listener code with default loop rate = 10Hz.
To change the loop rate give:

```
roslaunch beginner_tutorials basic.launch looprate:=1
```
This changes the loop rate of the talker to 1Hz. In both cases, the listener output opens in a new terminal. All terminals can be killed using ctrl+c.

## To use the service
The ROS service inside talker allows the user to change the text that is published from a default text. To run the service in a new terminal
```
rosservice call /changePublishedString <your string>
```
