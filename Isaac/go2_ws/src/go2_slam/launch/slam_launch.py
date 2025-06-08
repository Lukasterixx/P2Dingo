from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('go2_slam'),
            'config',
            'slam.yaml'
        ]),
        description='Full path to the slam_toolbox params file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock from simulation'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[ slam_params_file, { 'use_sim_time': use_sim_time } ]
    )

    return LaunchDescription([
      use_sim_time_arg,
      slam_params_file_arg,
      slam_node
    ])
