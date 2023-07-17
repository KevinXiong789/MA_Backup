# Standard library:
import os

# ament:
from ament_index_python import get_package_share_directory

# ROS-launch:
import launch
import launch_ros.actions
import launch.actions
import launch.launch_description_sources
import launch.event_handlers

def generate_launch_description():

    ld = launch.LaunchDescription()

    package_share_directory = get_package_share_directory(package_name='lop2')
    rviz_config_file_name = 'start_ur'
    
    # Add execute process for launching ur_robot_driver
    ur_robot_driver_launch = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur5e', 'robot_ip:=yyy.yyy.yyy.yyy',
             'use_fake_hardware:=true', 'launch_rviz:=false', 'initial_joint_controller:=joint_trajectory_controller'],
        output='screen'
    )
    ld.add_action(ur_robot_driver_launch)

    
    # Add execute process for launching ur_moveit_config
    ur_moveit_config_launch = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur5e', 'launch_rviz:=true',
             'use_fake_hardware:=true'],
        output='screen'
    )
    ld.add_action(ur_moveit_config_launch)

    return ld