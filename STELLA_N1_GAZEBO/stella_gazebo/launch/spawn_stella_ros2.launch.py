from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stella_description'),
                'launch',
                'stella_urdf_navi.launch.py'
            ])
        ])
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stella_gazebo'),
                'launch',
                'spawn_stella.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        urdf_visualize_launch,
        spawn_robot_launch,
        Node(
            package='stella_gazebo',
            executable='imu_to_ahrs',
            name='imu_to_ahrs',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='stella_gazebo',
            executable='odom_ahrs_fusion',
            name='odom_ahrs_fusion',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True}],
        ),
    ])