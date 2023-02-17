#!/bin/bash

source ~/vicon_ws/devel/setup.bash 

rosservice call /zero_joint 4
rosservice call /zero_joint 5
rosservice call /zero_joint 6
rosservice call /zero_joint 7