#!/bin/bash

source ~/vicon_ws/devel/setup.bash 

rosservice call /zero_joint 0
rosservice call /zero_joint 1
rosservice call /zero_joint 2
rosservice call /zero_joint 3