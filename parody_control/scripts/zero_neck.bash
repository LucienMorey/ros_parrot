#!/bin/bash

source ~/vicon_ws/devel/setup.bash 

rosservice call /zero_joint 8
rosservice call /zero_joint 9
rosservice call /zero_joint 10
rosservice call /zero_joint 11
rosservice call /zero_joint 12