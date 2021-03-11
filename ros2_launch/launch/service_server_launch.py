from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
    ld = LaunchDescription()

    service_node = Node(
        package="ros2_service_server",
        executable="add_numbers_server"
    )
    
    ld.add_action(service_node)
    return ld