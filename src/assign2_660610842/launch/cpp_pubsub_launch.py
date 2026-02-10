from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assign2_660610842',    
            executable='talker',           
            name='publisher_node',
            output='screen',
            remappings=[
                ('/gossip_660610842', 'assignment5')
            ],
            parameters=[{
                'who': 'myROS',
                'speaking': 'say',
                'spk_msg': 'y so EZ'
            }]
        ),

        Node(
            package='assign2_660610842',
            executable='listener1',          
            name='subscriber_1',
            output='screen',
            remappings=[
                ('/gossip_660610842', 'assignment5')
            ]
        ),

        Node(
            package='assign2_660610842',
            executable='listener1',
            name='subscriber_2',
            output='screen',
            remappings=[
                ('/gossip_660610842', 'assignment5')
            ]
        ),
    ])
