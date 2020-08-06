#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    ws_dir = '/your_ros2_ws_path/'
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value='basalt_',
            description='Prefix for node names'),
        launch_ros.actions.Node(
            parameters=[
                {'calibration_file':
                 ws_dir + \
                 'basalt_ros2/config/t265_example_calib.json',
                 'vio_config_file': 
                 ws_dir + \
                 'basalt_ros2/config/t265_example_vio_config.json',
	             'extra_translation': [0.0, 0.0, 0.0],
                 # rotation is in format [w, x, y, z]
	             'extra_rotation': [0.0, 0.0, 0.0, 1.0]
                }],
#            prefix=['gdb -ex=r --args'],
            package='basalt_ros2', node_executable='basalt_vio_backend_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'vio'],
            remappings=[
                        ('imu', '/imu'),
                        ('optical_flow', '/optical_flow')
            ]
        ),
    ])
