# arm_ros2

## Packages

- [arm_ros2](packages/arm_ros2/)
- [arm_ros2_interfaces](packages/arm_ros2_interfaces/)

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
