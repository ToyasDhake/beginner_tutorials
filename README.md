[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ToyasDhake/beginner_tutorials/blob/master/LICENSE.txt)
# ROS TF, rosbag and unit tesing demo

## Overview

Project consist of 3 parts ros tf, unit testing and rosbag. 

ROS TF:- Flie called talker.cpp which will broadcast tf frame called talk with 
parent world.

rosbag:- rosbag is used to record all the broadcasted messages and play it back 
when required. Frames from talker.cpp are recorded for ~15sec and then played 
back for listener.cpp.

Unit testing:- level 2 unit testing is done using gtest suite.

## Standard install via command-line

Run talker.cpp

```
git clone https://github.com/ToyasDhake/beginner_tutorials.git
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace>
catkin_make
source ./devel/setup.bash
roslaunch beginner_tutorials service.launch record_flag:=false
```
Setting record_flag to true will result in rosbag recording all the messages 
that are begin broadcasted.

Playback rosbag data and execute listener

```
(terminal 1)
git clone https://github.com/ToyasDhake/beginner_tutorials.git
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace>
catkin_make
roscore
(terminal 2)
cd <catkin_workspace>
source ./devel/setup.bash
rosrun beginner_tutorials listener 
(terminal 3)
cd <catkin_workspace/src/beginner_tutorials/results/>
rosbag play talkerData.bag 
```

Unit testing

```
git clone https://github.com/ToyasDhake/beginner_tutorials.git
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace>
catkin_make tests
catkin_make test
```

##Assumption

System has working installation of ROS kinetic.


## Copyright

Copyright (C) 2019 Toyas Dhake For license information, see LICENSE.txt.
