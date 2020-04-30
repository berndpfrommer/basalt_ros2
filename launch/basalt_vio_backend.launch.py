#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value='basalt_',
            description='Prefix for node names'),
        launch_ros.actions.Node(
            parameters=[{'calibration_file': '/your_path/calibration.json'}],
#            prefix=['gdb -ex=r --args'],
            package='basalt_ros2', node_executable='basalt_vio_backend_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'vio'],
            remappings=[
                        ('imu', '/imu'),
                        ('optical_flow', '/optical_flow')
            ]
        ),
    ])
