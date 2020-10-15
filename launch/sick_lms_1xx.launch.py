import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('sick_scan2'),
        'config',
        'sick_lms_1xx.yaml'
        )

    node=Node(
        package='sick_scan2',
        name = 'sick_scan2', # 'sick_scan2_lms_1xx', # For compatibility with ros versions previous to foxy, node name changed to sick_scan2 for all supported scanner. The type of scanner is configured by scanner_name in the yaml config file.
        node_executable='sick_generic_caller',       # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable".
        # executable='sick_generic_caller',          # Please use executable='sick_generic_caller', if ROS2 can't launch sick_generic_caller.
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
