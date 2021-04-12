# Differential Driving Robot with ESP32 running micro-ROS

The goal of this project is to build a differential drive robot that is controlled
by an ESP32 running [micro-ROS](https://github.com/micro-ROS) and is driven by an
[ODrive Motor Controller](https://odriverobotics.com/). Eventually the
[nav2](https://github.com/ros-planning/navigation2) stack will be used.

## TODO

- Create task for reading and writing to the ODrive
- Publish ODrive positions and velocities to `/joint_state`
- Create a `robot_state_publisher` launch file for use rviz2
- Subscribe to `/cmd_vel` and implement forward kinematics
- Tele-op control from `/joy`
- Publish Pose from wheel movement estimations
- Implement sensors for costmaps

## Build and Running Steps

### Build/Flash Firmware
Make sure the [micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) sub-module is updated

``` sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32
idf.py menuconfig
idf.py flash
```

### Launch micro-ROS agent
This needs to be done in a separate directory according to the
[Creating the micro-ROS agent](https://micro.ros.org/docs/tutorials/core/first_application_linux/) instructions.
```sh
source /opt/ros/foxy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```