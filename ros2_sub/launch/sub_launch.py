from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
    ld = LaunchDescription()

    sub_node = Node(
        package="ros2_sub",
        executable="number_subscriber"
    )

    ld.add_action(sub_node)
    return ld