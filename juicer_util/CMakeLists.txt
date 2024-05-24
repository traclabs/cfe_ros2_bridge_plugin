cmake_minimum_required(VERSION 3.5)
project(juicer_util)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

#include_directories(include)
#add_executable(cpp_executable src/cpp_node.cpp)
#ament_target_dependencies(cpp_executable rclcpp)
#install(TARGETS
#  cpp_executable
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/generate_juicer_database.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
