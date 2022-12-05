from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    name_topic = DeclareLaunchArgument('name_of_topic',default_value = TextSubstitution(text="publish_details"))
    publish_interval = DeclareLaunchArgument('publish_interval', default_value = TextSubstitution(text="1000"))
    # target_frame = DeclareLaunchArgument('target_frame', default_value = TextSubstitution(text="talk"))

    return LaunchDescription([
        name_topic,
        publish_interval,
        Node(
            package='week11_hw',
            executable='publish_details',
            name = "braodcaster1",
            parameters=[
                {"name_of_topic": LaunchConfiguration('name_of_topic')},
                {"publishing_interval": LaunchConfiguration('publish_interval')},
                {"frame_name" : 'talk'} 
            ]
        ),
        # DeclareLaunchArgument(
        #     'target_frame', default_value='talk',
        #     description='Target frame name.'
        # ),
        # Node(
        #     package='week11_hw',
        #     executable='publish_details',
        #     name = 'broadcaster2',
        #     parameters=[
        #         {"name_of_topic": LaunchConfiguration('name_of_topic')},
        #         {"publishing_interval": LaunchConfiguration('publish_interval')},
        #         {"frame_name" : 'listener'} 
        #     ]
        # ),
        # Node(
        #     package='week11_hw',
        #     executable='subscribe_details',
        #     parameters=[
        #         {'target_frame': LaunchConfiguration('target_frame')}
        #     ]
        # ),
    ])