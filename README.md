# ROS Tutorial on Publishers and Subscribers

## Overview

Beginner tutorial for publisher and subscriber mechanisum demo. 
This demo will run roscore first which act like a broker establishing 
communication between publisher and subscriber. Then it will run talker which 
will continuously publish msg "Hello I am Toyas Dhake <count>". Then it will run
listner which will recieve this message.

## Standard install via command-line

```
git clone --recursive " "
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace/build>
make
cd ..
(in terminal 1)
roscore
(in terminal 2)
source ./devel/setup.bash
rosrun beginner_tutorial talker
(in terminal 3)
source ./devel/setup.bash
rosrun beginner_tutorial listener
```

