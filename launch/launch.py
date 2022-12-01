from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    name_topic = DeclareLaunchArgument('name_of_topic',default_value = TextSubstitution(text="publish_details"))
    publish_interval = DeclareLaunchArgument('publish_interval', default_value = TextSubstitution(text="1000"))

    return LaunchDescription([
        name_topic,
        publish_interval,
        Node(
            package='week10_hw',
            executable='publish_details',
            parameters=[
                {"name_of_topic": LaunchConfiguration('name_of_topic')},
                {"publishing_interval": LaunchConfiguration('publish_interval')}
            ]
        ),
        Node(
            package='week10_hw',
            executable='subscribe_details'
        )
    ])