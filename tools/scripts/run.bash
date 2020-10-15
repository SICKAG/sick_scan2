#!/bin/bash

#
# Set environment
#

pushd ../../../..
source ./install/setup.bash

# ros2 launch sick_scan2 sick_tim_5xx.launch.py
ros2 launch sick_scan2 sick_ldmrs.launch.py &
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world laser &
ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_ldmrs.rviz 

popd

