#!/bin/bash

#
# Set environment
#

./killall.bash
printf "\033c"
pushd ../../../..
source ./install/setup.bash

#
# Run LDMRS simulation:
# 1. Start LDRMS test server
# 2. Start LDMRS driver
# 3. Run rviz
# 4. Stop simulation after 20 seconds
#

sleep  1 ; ros2 run sick_scan2 test_server --ros-args --params-file src/sick_scan2/tools/test_server/config/test_server_ldmrs.yaml &
sleep  1 ; ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_ldmrs_flexres.yaml -p "hostname:=127.0.0.1" &
sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_ldmrs.rviz &
sleep 20 ; ros2 topic echo diagnostics > ./log/sick_ldmrs_diagnostics.log &
sleep  1 ; ./src/sick_scan2/tools/scripts/killall.bash
sleep  1 ; tail -n 62 ./log/sick_ldmrs_diagnostics.log
sleep 3

#
# Run Cola based simulation:
# 1. Start Cola test server
# 2. Start TIM/LMS/MRS driver
# 3. Run rviz
# 4. Stop simulation after 10 seconds
#

for yaml_file in sick_tim_240.yaml sick_tim_5xx.yaml sick_tim_7xx.yaml sick_tim_7xxS.yaml sick_lms_1xx.yaml sick_lms_5xx.yaml sick_mrs_1xxx.yaml ; do
  sleep  1 ; ros2 run sick_scan2 test_server --ros-args --params-file src/sick_scan2/tools/test_server/config/test_server_cola.yaml &
  sleep  1 ; ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/$yaml_file -p "hostname:=127.0.0.1" -p "port:=2112" -p "sw_pll_only_publish:=false" & # -p "use_binary_protocol:=true" &
  sleep  1 ; ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_cola.rviz &
  sleep 10 ; ./src/sick_scan2/tools/scripts/killall.bash
done

# ros2 run sick_scan2 test_server --ros-args --params-file src/sick_scan2/tools/test_server/config/test_server_cola.yaml
# ros2 run rviz2 rviz2 -d ./src/sick_scan2/launch/rviz/sick_cola.rviz &
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_tim_240.yaml  -p "hostname:=127.0.0.1" -p "port:=2112"
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_tim_5xx.yaml  -p "hostname:=127.0.0.1" -p "port:=2112"
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_tim_7xx.yaml  -p "hostname:=127.0.0.1" -p "port:=2112"
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_tim_7xxS.yaml -p "hostname:=127.0.0.1" -p "port:=2112"
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_lms_1xx.yaml  -p "hostname:=127.0.0.1" -p "port:=2112"
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_lms_5xx.yaml  -p "hostname:=127.0.0.1" -p "port:=2112"
# ros2 run sick_scan2 sick_generic_caller --ros-args --params-file src/sick_scan2/config/sick_mrs_1xxx.yaml -p "hostname:=127.0.0.1" -p "port:=2112"
# ./src/sick_scan2/tools/scripts/killall.bash

popd

