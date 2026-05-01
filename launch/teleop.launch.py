from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type [burger, waffle, waffle_pi]'
    )

    teleop = Node(
        package='cpe631_ros2',
        executable='cpe631_teleop',
        parameters=[{'model': LaunchConfiguration('model')}],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([model_arg, teleop])
