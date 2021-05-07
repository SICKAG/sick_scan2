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


#uncomment this lines and change path accordingly
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
#to fix
#CMake Error at /snap/clion/126/bin/cmake/linux/share/cmake-3.17/Modules/FindPackageHandleStandardArgs.cmake:164 (message):
#  Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)


# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)
# find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  

add_definitions(-DUSE_DYN_RECONFIG=0)

if(BUILD_WITH_LDMRS_SUPPORT OR BUILD_WITH_LDMRS_SUPPORT MATCHES "true")
        message(STATUS "Building sick_scan2 with LDMRS support")
        add_definitions(-DLDMRS_SUPPORT=1)
        find_package(rosidl_default_generators REQUIRED)
        find_package(SickLDMRS REQUIRED)
        rosidl_generate_interfaces(${PROJECT_NAME}
            "msg/ldmrs/SickLdmrsObject.msg"
            "msg/ldmrs/SickLdmrsObjectArray.msg"
            DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
        )
        find_package(PCL REQUIRED) # workaround https://github.com/ros2/rosidl/issues/402 : Do find_package(PCL) after calling rosidl_generate_interfaces in your CMakeLists.txt
        set(LDMRS_INCLUDES ${SICK_LDMRS_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
        set(LDMRS_SOURCES driver/src/ldmrs/sick_ldmrs_config.cpp driver/src/ldmrs/sick_ldmrs_driver.cpp driver/src/ldmrs/sick_ldmrs_node.cpp)
        set(LDMRS_TARGET_DEPENDENCIES SickLDMRS)
        message(STATUS "LDMRS_INCLUDES: ${LDMRS_INCLUDES}")
        message(STATUS "LDMRS_SOURCES: ${LDMRS_SOURCES}")
        message(STATUS "LDMRS_TARGET_DEPENDENCIES: ${LDMRS_TARGET_DEPENDENCIES}")
endif()

find_package(Boost REQUIRED COMPONENTS system thread) # rosidl_generate_interfaces overwrites ${Boost_LIBRARIES}. Workaround: Do find_package(Boost ) after calling rosidl_generate_interfaces in CMakeLists.txt

if(BUILD_WITH_TEST_SERVER OR BUILD_WITH_TEST_SERVER MATCHES "true")
        message(STATUS "Building sick_scan2 with test server support")
        set(TEST_SERVER_INCLUDES tools/test_server/include) 
endif()

include_directories(include ${Boost_INCLUDE_DIR} ${LDMRS_INCLUDES} ${TEST_SERVER_INCLUDES})

add_library(sick_scan2_lib
        driver/src/sick_generic_laser.cpp
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
        driver/src/binPrintf.cpp
        driver/src/binScanf.cpp
        driver/src/sick_scan_common_nw.cpp
        driver/src/sick_generic_imu.cpp
        driver/src/softwarePLL.cpp
        ${LDMRS_SOURCES}
) 

ament_target_dependencies(
        sick_scan2_lib
        "Boost"
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs" 
        "tf2_ros" 
        ${LDMRS_TARGET_DEPENDENCIES}
)

add_executable(sick_generic_caller
        driver/src/sick_generic_caller.cpp
        )

ament_target_dependencies(
        sick_generic_caller
        "Boost"
        # "diagnostic_updater"
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "diagnostic_msgs" 
        "tf2_ros" 
        ${LDMRS_TARGET_DEPENDENCIES}
)

#add_executable(quat_test
#        tools/quat_test/quat_test.cpp
#        include/sick_scan/sick_generic_imu.h
#        include/sick_scan/softwarePLL.h)

#ament_target_dependencies(quat_test
#        "rclcpp"
#        "tf2"
#        )

if(BUILD_WITH_LDMRS_SUPPORT OR BUILD_WITH_LDMRS_SUPPORT MATCHES "true")
        rosidl_target_interfaces(sick_scan2_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
        rosidl_target_interfaces(sick_generic_caller ${PROJECT_NAME} "rosidl_typesupport_cpp")
        target_link_libraries(sick_generic_caller sick_scan2_lib ${SICK_LDMRS_LIBRARIES} ${Boost_LIBRARIES})
else()        
        target_link_libraries(sick_generic_caller sick_scan2_lib)
endif()

if(BUILD_WITH_TEST_SERVER OR BUILD_WITH_TEST_SERVER MATCHES "true")
        add_executable(test_server 
            tools/test_server/src/test_server.cpp 
            tools/test_server/src/test_server_cola_msg.cpp 
            tools/test_server/src/test_server_ldmrs_msg.cpp 
            tools/test_server/src/test_server_thread.cpp)
        target_link_libraries(test_server sick_scan2_lib ${SICK_LDMRS_LIBRARIES} ${Boost_LIBRARIES})
        ament_target_dependencies(test_server "rclcpp")
        install(TARGETS 
            test_server
            DESTINATION lib/${PROJECT_NAME})
        install(DIRECTORY 
            tools/test_server/config
            tools/test_server/launch
            DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS
  sick_generic_caller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  sick_scan2_lib
  DESTINATION lib)

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
