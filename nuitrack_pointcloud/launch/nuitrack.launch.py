# Standart library:
import os

# ament:
from ament_index_python import get_package_share_directory

# ROS-launch:
import launch
import launch_ros.actions
import launch.actions
import launch.launch_description_sources

from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess



def generate_launch_description():
	ld = launch.LaunchDescription()

	
	rviz_node = launch_ros.actions.Node(
		name='rviz',
		package='rviz2',
		namespace='lop_rviz',
		executable='rviz2',
		#arguments=['-d' + os.path.join(package_share_directory, 'config', f'{rviz_config_file_name}.rviz'), '--fixed-frame', 'world' ]
	)
	#ld.add_action(rviz_node)
	
	nuitrack_node = launch_ros.actions.Node(
		package='nuitrack_pointcloud',
		executable='nuitrack_skeleton_pointcloud',
		output='screen'
	)
	ld.add_action(nuitrack_node)
	
	static_transform_publisher  = launch_ros.actions.Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=['0.0', '0.0', '0.0',
				'0.0', '0.0', '0.0',
				'world','camera_link'],
		#output='screen'
	)
	ld.add_action(static_transform_publisher)
	

	# Add execute process for launching ur_robot_driver
	ur_robot_driver_launch = launch.actions.ExecuteProcess(
		cmd=['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur10e', 'robot_ip:=yyy.yyy.yyy.yyy',
			 'use_fake_hardware:=true', 'launch_rviz:=false', 'initial_joint_controller:=joint_trajectory_controller'],
		output='screen'
	)
	ld.add_action(ur_robot_driver_launch)

	
	# Add execute process for launching ur_moveit_config
	ur_moveit_config_launch = launch.actions.ExecuteProcess(
		cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur10e', 'launch_rviz:=true',
			 'use_fake_hardware:=true'],
		output='screen'
	)
	ld.add_action(ur_moveit_config_launch)
	
	
	# Define a timer to trigger the execution of a C++ file after 10 seconds
	handover_after10 = TimerAction(period=10.0, actions=[
		ExecuteProcess(
			cmd=['ros2', 'run', 'pointcloud_processing_nuitrack', 'handover_T_inhand_detector'],
			output='screen'
		)
	])
	ld.add_action(handover_after10)

	
	return ld