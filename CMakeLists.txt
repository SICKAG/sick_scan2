###########################################################
# ROS2 CMakeLists.txt for SICK-Scanner-Support
#
# Project: sick_scan2
# URL: https://github.com/SICKAG/sick_scan2
#
###########################################################

cmake_minimum_required(VERSION 3.5)

project(sick_scan2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# TODO: Remove compiler warning from the code - currently suppressed

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-sign-compare )
#  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)
# find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_definitions(-DUSE_DYN_RECONFIG=0)


include_directories(include ${Boost_INCLUDE_DIR})

add_executable(sick_generic_caller
        driver/src/sick_generic_laser.cpp
        driver/src/sick_generic_caller.cpp
        driver/src/sick_generic_parser.cpp
        driver/src/sick_scan_common_tcp.cpp
        driver/src/sick_scan_common.cpp
        driver/src/tcp/tcp.cpp
        driver/src/tcp/Mutex.cpp
        driver/src/tcp/SickThread.cpp
        driver/src/tcp/errorhandler.cpp
        driver/src/tcp/toolbox.cpp
        driver/src/tcp/Time.cpp
        driver/src/tcp/colaa.cpp
        driver/src/tcp/colab.cpp
        driver/src/binScanf.cpp
        driver/src/sick_scan_common_nw.cpp
        )

ament_target_dependencies(sick_generic_caller
   "Boost"
#   "diagnostic_updater"
   "rclcpp"
   "sensor_msgs"
   "std_msgs"
)

add_executable(quat_test
  tools/quat_test/quat_test.cpp
)

ament_target_dependencies(quat_test
        "rclcpp"
        "tf2"
        )

install(TARGETS
  sick_generic_caller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
