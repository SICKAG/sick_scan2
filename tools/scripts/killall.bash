#!/bin/bash

sleep 1 ; killall -SIGINT rviz2
sleep 1 ; killall -SIGINT sick_generic_caller
sleep 1 ; killall -SIGINT test_server
sleep 1 ; killall -9 rviz2
sleep 1 ; killall -9 sick_generic_caller
sleep 1 ; killall -9 test_server
sleep 1 

