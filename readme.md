## Some tips and notes made during the ros2 practice session

## ros2_interface
- msg and srv should follow strictly `SomeMsg.msg/SomeSrv.srv`
- use ament_cmake build type for the interfaces pkg.

## ros2_pub & ros2_sub
- always add the pub and sub to the setup.py 
```
'node_name=pkg_name.file_name:ros2_function_name'
```
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