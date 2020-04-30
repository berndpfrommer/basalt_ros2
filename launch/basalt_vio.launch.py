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
            parameters=[{'calibration_file':
                         '/your_directory/' + \                         
                         'basalt_ros2/config/calibration.json',
                         'debug_vio': False,
                         'debug_bad_data': False}],
#            prefix=['gdb -ex=r --args'],
            package='basalt_ros2', node_executable='basalt_vio_node', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'vio'],
            remappings=[('/left_image', '/t265/camera/fisheye1/image_raw'),
                        ('/right_image', '/t265/camera/fisheye2/image_raw'),
                        ('/gyro', '/t265/camera/gyro/sample'),
                        ('/accel', '/t265/camera/accel/sample'),
                        ('odom', '/basalt_vio/odom')
            ]
        ),
    ])
