cmake_minimum_required(VERSION 3.5)
project(contactile_gripper)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include/${PROJECT_NAME}
)

# Add message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float32List.msg"
  "msg/Int32List.msg"
  "srv/ChangeMode.srv"
  "srv/GripperChangeMode.srv"
  "srv/StepperSetLimit.srv"
  "srv/StepperOff.srv"
  "srv/DataRecorder.srv"
  "srv/UIMenu.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

install(DIRECTORY
  launch
  support
  src
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  launch/autogen.launch
  src/camera_node.py
  src/control_node.py  
  src/data_recorder_node.py
  src/gripper_node.py
  src/IMU_node.py
  src/pose_node.py
  src/srv_clients.py
  src/stepper_node.py
  src/sys_test_node.py
  src/ui_node.py

  # support/camera.py
  # support/dynamixel_motors.py
  # support/get_comport_info.py
  # support/gripper.py
  # support/IMU.py
  # support/pose_models.py
  # support/rclpy_shadow.py
  # support/stepper.py

  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables from the src directory
ament_python_install_package(src)

# Declare this package as a ROS 2 package
ament_package()
