#!/bin/bash

#
# Set environment
#

pushd ../../../..
source ./install/setup.bash

ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_ldmrs.yaml -p "hostname:=127.0.0.1" &
ros2 launch sick_scan2 sick_ldmrs.launch.py  --ros-args -p hostname:=127.0.0.1 &
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world laser &
ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_ldmrs.rviz 

popd

