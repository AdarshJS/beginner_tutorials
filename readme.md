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

## Using rosservice to change published string
The ROS service inside talker allows the user to change the text that is published from a default text. To run the service in a new terminal
```
rosservice call /changePublishedString <your string>
```

## tf Broadcaster
The talker node also broadcasts a transform between two frames: /world and /talk. The transform has a translation of 3 units in X, 4 units in Y and 0 units in Z directions, and a rotational components of 90 degrees about the Z axis. Use tf_echo to view the translation and rotation transform vectors. Run the talker node and then :

```
rosrun tf tf_echo /world /talk
```

To see a diagram of how the two frames are connected, run:
```
rosrun tf view_frames
```
This produces a PDF in the directory in which it is run. View the PDF by running:
```
cd <directory in which view_frames was run>
evince frames.pdf
```
See results folder for an example of the generated PDF.

## Using ROSbag
The rosbag file (.bag) that recorded all published topics is in the results folder. To run it, make sure ros master is running:
```
source ~/.bashrc
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play output.bag
```  
To see what is published, run the listener node as:
```
rosrun beginner_tutorials listener
```

This will display the strings published in the /chatter topic.
To create a new rosbag file run:
```
roslaunch beginner_tutorials basic.launch runRosbag:=true
```
A new rosbag is stored as output.bag inside ~/.ros directory.

## ROSTEST
To build the tests, run:
```
cd ~/catkin_ws
catkin_make run_tests
```
To run the rostest that tests the services in the talker node, run:
```
rostest beginner_tutorials test.launch
```

This should produce an output such as :
```
[ROSUNIT] Outputting test results to /home/adarshjs/.ros/test_results/beginner_tutorials/rostest-test_test.xml
[Testcase: testrosTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-rosTest/testServiceExistence][passed]
[beginner_tutorials.rosunit-rosTest/testStringUpdate][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
 rostest log file is in /home/adarshjs/.ros/log/rostest-Friday-11661.log
```
