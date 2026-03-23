import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ekf_config = '/home/robot/ros2_ws/src/robot_driver/config/ekf.yaml'
    slam_config = '/home/robot/ros2_ws/src/robot_driver/config/slam_params.yaml'
    
    robot_driver_node = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver_node',
        output='screen'
    )

    gy25_driver_node = Node(
        package='robot_driver',
        executable='gy25_driver_node',
        name='gy25_driver_node',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros2_driver'), 'launch'),
            '/rplidar_with_args.launch.py']
        ),
        launch_arguments={
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'serial_baudrate': '115200',
            'scan_mode': 'Standard',
            'publish_tf': 'false'
        }.items()
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        robot_driver_node,
        gy25_driver_node,
        ekf_node,
        lidar_launch,
        slam_node
    ])
