from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
    ld = LaunchDescription()

    client_node = Node(
        package="ros2_service_client",
        executable="async_client",

    )
    
    ld.add_action(client_node)
    return ld