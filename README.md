[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ToyasDhake/beginner_tutorials/blob/Week10_HW/LICENSE.txt)
# ROS service for a simple caluctor

## Overview

This demonstration of ROS service/client mechanism. The client connects to the 
service and sends requests to perform operations. The service can perform four 
basic operation right now: addition, subtraction, multiplication and division.

Client sends three things one operator and two operand.

## Standard install via command-line

```
git clone https://github.com/ToyasDhake/beginner_tutorials.git
cp <path to repository> <catkin_workspace/src/>
cd <catkin_workspace/build>
make
cd ..
source ./devel/setup.bash
roslaunch --screen beginner_tutorials service.launch speed:="<argument>"
-Enter operator (ADD, SUB, MUL, DIV)
-Enter first number
-Enter second number
```
Argument can be either "slow" or "fast". Slow will result in service running 10
time a second and fast will result in service running 30 times a second.

## Limitations
- Calcuator cannot perform advance operations. It is limited to 4 operations:-
addition, subtraction, multiplicaiton and division.

Note: ROS service accepts adress by reference by using '&' but cpplint is giving 
error saying it should be either decleared const or use pointer. Both are not 
accepted by ROS.

## Copyright

Copyright (C) 2019 Toyas Dhake For license information, see LICENSE.txt.
