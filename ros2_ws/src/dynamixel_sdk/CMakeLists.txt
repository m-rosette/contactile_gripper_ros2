################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include/${PROJECT_NAME}
)

set(DYNAMIXEL_SDK_SOURCES
  src/dynamixel_sdk/packet_handler.cpp
  src/dynamixel_sdk/protocol1_packet_handler.cpp
  src/dynamixel_sdk/protocol2_packet_handler.cpp
  src/dynamixel_sdk/group_sync_read.cpp
  src/dynamixel_sdk/group_sync_write.cpp
  src/dynamixel_sdk/group_bulk_read.cpp
  src/dynamixel_sdk/group_bulk_write.cpp
  src/dynamixel_sdk/port_handler.cpp
)

if(APPLE)
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/dynamixel_sdk/port_handler_mac.cpp
  )
elseif(WIN32)
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/dynamixel_sdk/port_handler_windows.cpp
  )
else()
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/dynamixel_sdk/port_handler_linux.cpp
  )
endif()

################################################################################
# Install
################################################################################
# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY scripts/
  DESTINATION scripts/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# Setup for python modules and scripts
################################################################################
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  scripts/group_bulk_read.py
  scripts/group_bulk_write.py
  scripts/group_sync_read.py
  scripts/group_sync_write.py
  scripts/packet_handler.py
  scripts/port_handler.py
  scripts/protocol1_packet_handler.py
  scripts/protocol2_packet_handler.py
  scripts/robotis_def.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)

ament_export_include_directories(include)
ament_export_include_directories(scripts)

ament_export_libraries(${PROJECT_NAME})

ament_package()
