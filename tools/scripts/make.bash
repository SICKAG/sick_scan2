#!/bin/bash

#
# Set environment
#

pushd ../../../..
if [ -f /opt/ros/eloquent/setup.bash ] ; then source /opt/ros/eloquent/setup.bash ; fi
if [ -f /opt/ros/foxy/setup.bash ]     ; then source /opt/ros/foxy/setup.bash     ; fi
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f ./log/sick_scan2_build_errors.log ] ; then rm -f ./log/sick_scan2_build_errors.log ; fi

#
# set build type (Debug or Release)
#
# BUILDTYPE=Debug
# BUILDTYPE=Release
 
#
# Build and install sick_scan2 binaries without LDMRS and test support.
#

if [ 1 == 0 ] ; then
  unset BUILD_WITH_LDMRS_SUPPORT
  unset BUILD_WITH_TEST_SERVER
  if [ -f ./build_std/sick_scan2/sick_generic_caller ] ; then rm -f ./build_std/sick_scan2/sick_generic_caller ; fi
  colcon build --event-handlers console_direct+
  # colcon build --build-base ./build_std --install-base ./install_std --symlink-install --event-handlers console_direct+ # --cmake-args " -DCMAKE_BUILD_TYPE=$BUILDTYPE"
  echo -e "colcon build warnings and errors (LDMRS and test support disabled):" >> ./log/sick_scan2_build_errors.log
  cat ./log/latest_build/*.* ./log/latest_build/*/*.* | grep -i "warning:"      >> ./log/sick_scan2_build_errors.log
  cat ./log/latest_build/*.* ./log/latest_build/*/*.* | grep -i "error:"        >> ./log/sick_scan2_build_errors.log
  source ./install_std/setup.bash
fi

#
# Build and install sick_scan2 binaries with LDMRS and test support.
#

if [ 1 == 1 ] ; then
  export BUILD_WITH_LDMRS_SUPPORT=True
  export BUILD_WITH_TEST_SERVER=True
  if [ -f ./build/sick_scan2/sick_generic_caller ] ; then rm -f ./build/sick_scan2/sick_generic_caller ; fi
  colcon build --event-handlers console_direct+ --cmake-args " -DBUILD_WITH_LDMRS_SUPPORT=$BUILD_WITH_LDMRS_SUPPORT" " -DBUILD_WITH_TEST_SERVER=$BUILD_WITH_TEST_SERVER" # " -DCMAKE_BUILD_TYPE=$BUILDTYPE"
  echo -e "colcon build warnings and errors (LDMRS and test support enabled):" >> ./log/sick_scan2_build_errors.log
  cat ./log/latest_build/*.* ./log/latest_build/*/*.* | grep -i "warning:"     >> ./log/sick_scan2_build_errors.log
  cat ./log/latest_build/*.* ./log/latest_build/*/*.* | grep -i "error:"       >> ./log/sick_scan2_build_errors.log
  source ./install/setup.bash
fi

# print warnings and errors
echo -e "\nmake.bash finished.\n"
echo -e "\ncolcon build warnings and errors:"
cat ./log/sick_scan2_build_errors.log
if [ ! -f ./install/libsick_ldmrs/lib/libsick_ldmrs.so.0.1.0 ] ; then echo -e "\n## ERROR building libsick_ldmrs/lib/libsick_ldmrs.so\n"                               ; fi
if [ ! -f ./install/libsick_ldmrs/bin/LDMRS_Example ]          ; then echo -e "\n## ERROR building libsick_ldmrs/lib/LDMRS_Example\n"                                  ; fi
if [ ! -f ./build/sick_scan2/sick_generic_caller ]             ; then echo -e "\n## ERROR building sick_scan2/sick_generic_caller (LDMRS and test support enabled)\n"  ; fi
# if [ ! -f ./build_std/sick_scan2/sick_generic_caller ]       ; then echo -e "\n## ERROR building sick_scan2/sick_generic_caller (LDMRS and test support disabled)\n" ; fi

# print sick_scan2 install files, libraries, executables
echo -e "\nbuild sick_scan2 finished:"
ls -al ./install/libsick_ldmrs/lib/libsick_ldmrs.so.0.1.0 ./install/libsick_ldmrs/bin/LDMRS_Example ./build/sick_scan2/sick_generic_caller ./build/sick_scan2/test_server
echo -e "\n"
popd

