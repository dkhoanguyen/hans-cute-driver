# Set the project name
cmake_minimum_required(VERSION 3.0.2)
project(serial_port)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(${PROJECT_NAME}_SOURCES
  src/custom_serial_port.cpp
  src/dummy_serial_port.cpp
)

# Add a library with the above sources
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)