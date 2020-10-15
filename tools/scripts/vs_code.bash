#!/bin/bash

# 
# Start Visual Studio Code
# 

echo -e "\nvs_code.bash: Starting visual studio code."  
# echo -e "set BUILDTYPE=Debug in make.bash for debugging and development."  
# echo -e "set BUILDTYPE=Release in make.bash for profiling and benchmarks.\n"  
gedit ./make.bash ./run.bash ./run_simu.bash &

pushd ../../../..
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash ]     ; then source /opt/ros/foxy/setup.bash     ; fi
source /opt/ros/$ROS_DISTRO/setup.bash 
source ./install/setup.bash
code ./sick_scan2_vscode.code-workspace 
popd
