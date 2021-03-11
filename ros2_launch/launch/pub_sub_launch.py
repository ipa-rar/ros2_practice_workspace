from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
    ld = LaunchDescription()

    pub_node = Node(
        package="ros2_pub",
        executable="number_publisher"
    )
    
    sub_node = Node(
        package="ros2_sub",
        executable="number_subscriber"
    )
    
    ld.add_action(pub_node)
    ld.add_action(sub_node)

    return ld