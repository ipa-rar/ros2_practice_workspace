from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
    ld = LaunchDescription()

    pub_node = Node(
        package="ros2_pub",
        executable="number_publisher"
    )
    
    ld.add_action(pub_node)
    return ld