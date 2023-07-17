
# Standart library:
import os

# ament:
from ament_index_python import get_package_share_directory

# ROS-launch:
import launch
import launch_ros.actions
import launch.actions
import launch.launch_description_sources



#realsense_d435_serial_number =   '937622073146'
realsense_d435_serial_number = '838212073332'
#realsense_d435i_serial_number =  '923322070787'
realsense_d435i_serial_number =  '117322070913'
realsense_d455_1_serial_number = '213522254445'
realsense_d455_2_serial_number = '215122251396'

realsense_camera_serial_number = realsense_d435_serial_number


publish_images =  True
publish_markers = True

rviz_config_file_name = 'depth_only' if (not publish_images) and publish_markers \
                        else 'image_only' if publish_images and (not publish_markers) \
                        else 'marker'



def generate_launch_description():

    package_share_directory = get_package_share_directory(package_name='lop2')

    lop_node = launch_ros.actions.Node(
        name='lop',
        package='lop2',
        namespace='lop',
        executable='lop_node',
        parameters=[
            {'publish_images':  publish_images},
            {'publish_markers': publish_markers}
        ]
    )

    rviz_node = launch_ros.actions.Node(
        name='rviz',
        package='rviz2',
        namespace='lop_rviz',
        executable='rviz2',
        arguments=['-d' + os.path.join(package_share_directory, 'config', f'{rviz_config_file_name}.rviz') ]
    )

    realsense_launch = launch.actions.IncludeLaunchDescription(
        launch_description_source=launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory(package_name='realsense2_camera'), 'launch/rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'serial_no': f"'{realsense_camera_serial_number}'",  # Double quotes are required.
            'clip_distance': '5.0',  # Filter out depth data past this distance to the camera, in meters.
            'config_file': os.path.join(package_share_directory, 'data', 'realsense-config.yaml'),
        }.items()
    )

    return launch.LaunchDescription(initial_entities=[lop_node, rviz_node, realsense_launch] )

