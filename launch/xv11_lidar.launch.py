"""
Launch file for XV-11 Lidar driver in dev_ws project.

This launch file uses the xv11_lidar_python package from ros2_ws
and launches it from the dev_ws project.

Usage:
    cd ~/dev_ws
    ros2 launch launch/xv11_lidar.launch.py
    
    With custom parameters:
    ros2 launch launch/xv11_lidar.launch.py port:=/dev/ttyAMA0 frame_id:=xiaomi_lidar
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch XV-11 Lidar from ros2_ws using this dev_ws launcher."""

    # Get the xv11_lidar_python package from ros2_ws
    # Make sure ros2_ws is built and sourced in your environment
    try:
        xv11_pkg_dir = get_package_share_directory('xv11_lidar_python')
    except Exception as e:
        raise RuntimeError(
            "Could not find 'xv11_lidar_python' package. "
            "Make sure:\n"
            "1. ros2_ws/src/xv11_lidar_python is built\n"
            "2. You have sourced ros2_ws/install/setup.bash AFTER dev_ws/install/setup.bash\n"
            f"Error: {e}"
        )

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the lidar device (e.g., /dev/ttyUSB0, /dev/ttyAMA0)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='xv11_lidar',
        description='TF frame ID for the laser scan'
    )
    
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.06',
        description='Minimum range in meters'
    )
    
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='13.0',
        description='Maximum range in meters'
    )

    # Include the xv11_lidar launch file from ros2_ws
    xv11_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xv11_pkg_dir, 'launch', 'xv11_lidar.launch.py')
        ),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
        }.items(),
    )

    return LaunchDescription([
        port_arg,
        frame_id_arg,
        range_min_arg,
        range_max_arg,
        xv11_launch,
    ])
