cmake_minimum_required(VERSION 3.8)
project(arm_ros2_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	action/ArmControl.action
	msg/ArmStatus.msg
	msg/GripperStatus.msg
	msg/InverseKinematicsCalculatorResult.msg
	msg/JointStatus.msg
	srv/InverseKinematicsCalculator.srv
	DEPENDENCIES unique_identifier_msgs builtin_interfaces action_msgs)

ament_package()
