# arm_ros2

## Dependencies

### ROS 2 Distro

- Humble

### ROS 2 Packages

- [`ament_cmake`](https://github.com/ament/ament_cmake)
- [`rclcpp`](https://github.com/ros2/rclcpp)

### External packages

- [`yaml-cpp`](https://github.com/jbeder/yaml-cpp)

## Registered Topics / Services / Actions

| Type             | Name                            | Direction       | Message/Service Type                       | Description                                               |
| ---------------- | ------------------------------- | ----------------| ------------------------------------------ | --------------------------------------------------------- |
| Action           | `ArmControl`                    | Action Server   | `include/arm_ros2/ros/action/ArmControl`   | Handles mission arm movement requests                     |

## Build Instructions

To build the project, the following commands should be run directly from your ROS2 workspace:

```
colcon build
source install/setup.bash
```
