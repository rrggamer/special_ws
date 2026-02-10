from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
	Node(
	    package='cpp_pubsub',
	    executable='talker',
	    name='Publisher'
	    ),
	
	Node(
	    package='cpp_pubsub',
	    executable='listener',
	    name='Listener'
	    )
	]
    )

