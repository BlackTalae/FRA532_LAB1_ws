import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dataset_no = 0
    dataset_name = f"fibo_floor3_seq0{dataset_no}"
    
    # Get package directory
    package_dir = get_package_share_directory('fra532_lab1_package')
    
    # Construct relative paths
    dataset_path = os.path.join(package_dir, 'dataset', dataset_name)
    rviz_config_path = os.path.join(package_dir, 'config', '1_EKF_odom_config.rviz')
    rviz_compare_config_path = os.path.join(package_dir, 'config', '1_compare_EKF_wheel_odom_config.rviz')

    return LaunchDescription([

        # 1. EKF Odometry Node

        Node(
            package='fra532_lab1_package',
            executable='EKF_odom_node.py',
            name='EKF_odom_node',
        ),

        # 2. Rosbag
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', dataset_path, '--rate', '20.0'],
                    output='screen'
                ),
            ]
        ),
        
        # 3. RViz2
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_compare_config_path],
            output='screen'
        ),

        # Run compare to wheel odometry node
        Node(
            package='fra532_lab1_package',
            executable='wheel_odom_node.py',
            name='wheel_odom_node',
        ),
    ])