## Some tips and notes made during the ros2 practice session
## common
- Always add the nodes @entry_points of the setup.py 
```
'node_name=pkg_name.file_name:ros2_function_name'
```

## ros2_interface
- msg and srv should follow strictly `SomeMsg.msg/SomeSrv.srv`
- use ament_cmake build type for the interfaces pkg.

## ros2_pub & ros2_sub
- When using any msgs add them to the `package.xml` as `<depend>msg_type</depend>`
- To avoid error during a keyboard interrupt add an exception 
```
rclpy.init('node_name)
try:
    _node_ = NodeName()
    rclpy.spin(_node_)

except KeyboardInterrupt:
    _node_.destroy_node()
    rclpy.shutdown()
```

    - Create the node using py
    - package.xml for adding dependencies
    - setup.py for adding the pub and sub executable names

## ros2_service_client & ros2_service_server
-The type and name of the service must match for the client and service to communicate

## Writing ros2 Launch files
- Create a `/launch` folder inside the pkg and *.py file
- Boilerplate code for launch_file.py 
```
from launch import LaunchDescription
from launch_ros.actions import Node
    
    def generate_launch_description():
        ld = LaunchDescription()
        pub_node = Node(
            package="ros2_pub",
            executable="number_publisher"
        )
        sub_node = Node(
            package="ros2_pub",
            executable="number_subscriber"
        )

        ld.add_action(pub_node)
        ld.add_action(sub_node)

  return ld
```
- Add this line to Setup.py
```
    import os
    from glob import glob
    data_files=[
        ................................. ,
        ................................. ,
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
```
## Resolving dependencies

-       rosdep install -i --from-path src --rosdistro foxy -y


## Build pkgs

-       colcon build

- `--symlink-install` saves you from having to rebuild every time you tweak python scripts
- The `install` directory is where your workspace’s setup files are, which you can use to source your overlay.
- Source your underlay
-     source /opt/ros/foxy/setup.bash  

- Source your overlay 
-       . install/local_setup.bash
- Sourcing the `local_setup` of the overlay will only add the packages available in the overlay to your environment.
- `setup` sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.

## API [rclpy](https://docs.ros2.org/latest/api/rclpy/)
-       rcpy.create_node('node_name')
-       Node.create_publisher(publisher_handle,
                          msg_type
                          topic,
                          qos_profile)
-        publish(msg)

-       Node.creat_subscription(subscription_handle,
                                msg_type,
                                topic,
                                callback_function,
                                qos_profile  
                                )   
-       Node.create_service(service_handle,
                            srv_type,
                            srv_name,
                            qos_profile
                            )

-       send_response(response, header)

-       Node.create_client(cliet_handle,
        srv_type,
        srv_name,
        qos_profile)

        call(request)
        call_async(request)

-        wait_for_service(timeout_sec=None)