## ROS tutorials basics

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
git clone https://github.com/AdarshJS/beginner_tutorials.git
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
