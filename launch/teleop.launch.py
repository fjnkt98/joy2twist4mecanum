import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    config = launch.substitutions.LaunchConfiguration('config')
    device = launch.substitutions.LaunchConfiguration('device')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('config', default_value='f310'),
        launch.actions.DeclareLaunchArgument('device', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('joy2twist4mecanum'), 'config', ''
                )),
                config,
                launch.substitutions.TextSubstitution(text='.config.yaml')
            ]),

        launch_ros.actions.Node(
            package='joy', node_executable='joy_node', name='joy_node',
            parameters=[{
                'dev': device,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                }]
            ),
        launch_ros.actions.Node(
            package='joy2twist4mecanum', node_executable='joy2twist4mecanum_node',
            name='joy2twist4mecanum_node', parameters=[config_filepath],
            output='screen'
            ),
        ])
