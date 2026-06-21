Dependencies

pip install pyserial cobs crccheck protobuf

PYTHONUNBUFFERED=1 ros2 run your_package_name stm32_bridge_node


ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
