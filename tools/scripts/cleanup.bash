#!/bin/bash

./killall.bash
pushd ../../../../
source /opt/ros/eloquent/setup.bash
if [ -d build ] || [ -d install ] || [ -d log ] ; then rm -rf ./build ./install ./log ; fi
if [ -d build_std ] || [ -d install_std ] || [ -d log ] ; then rm -rf ./build_std ./install_std ./log ; fi
if [ ! -d ./install/sick_scan2    ] ; then mkdir -p ./install/sick_scan2    ; fi
if [ ! -d ./install/libsick_ldmrs ] ; then mkdir -p ./install/libsick_ldmrs ; fi
rm -rf ~/.ros/*
popd

