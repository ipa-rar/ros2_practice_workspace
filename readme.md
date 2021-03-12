## Some tips and notes for ros2 dummies
## common
- Always add the nodes @entry_points of the setup.py 
```
'node_name=pkg_name.file_name:ros2_function_name'
```

## ros2_interface
- A pkg to add new msg, srv or act interfaces used in this stack.
- This pkg uses `ament_cmake` build system as there is currently no way to generate .msg and .srv file in a pure python package
- msg and srv should follow strictly `SomeMsg.msg/SomeSrv.srv`
- Add these to the `package.xml` as they rely on `rosidl_default_generators` for generating language-specific code.
```
  <!-- This is added to generate msgs nd srv -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
- Add these lines to `CMakeLists.txt` to cnovert the interfaces you defined into language specific code.
```
# for generating msgs and srvs
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MathsMsg.msg"
  "srv/MathsSrv.srv"
 )
```

## ros2_pub & ros2_sub
- A pkg that publishes and subscribes `MathsMsg` over `/maths_topic`
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
- A pkg to demo ros2 service client and server. Client sends requests  and server responds back over `MathsSrv` service.
- The type and name of the service must match for the client and service to communicate.

## ros2_launch
### Launch files using python scripts
- Create a `/launch` folder inside the pkg create a `*.py` file

- Add this line to `Setup.py` so that colcon build can find it during the build process
```
    import os
    from glob import glob
    data_files=[
        ................................. ,
        ................................. ,
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
```
- Boilerplate code for `launch_file.py `
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

[launch files ref](https://git.fh-aachen.de/lm9299s/agv/-/tree/master/launch)
### Launch files using xml file
- Create a `.xml` file inside the `/launch` folder and use this as template. 
```
<launch>
    <node pkg="ros2_pub" exec="number_publisher" output="screen"/>
    <node pkg="ros2_sub" exec="number_subscriber" output="screen"/>
</launch>
```
- To launch the nodes use 
```
ros2 launch path/to/the/launch/file.xml
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

## ros2_linting
[ref to ament_linting](https://ubuntu.com/blog/how-to-add-a-linter-to-ros-2)

## TODOS
- ros2_action_server and client
- ros2_lifecycle
- Passing arguments via launch files. 
- parameters load and use
