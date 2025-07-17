cmake_minimum_required(VERSION 3.8)
project(bmi270_imu_node)

# Set C++ version
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 CONFIG REQUIRED) # for applying calibration result from imu_tk
# finds the bmi270_driver package and make its exported targets available to the node
find_package(bmi270_driver REQUIRED)

include_directories(
  # include
  ${Eigen3_INCLUDE_DIR} # Add Eigen's include directory
                        # on my rpi eigen is in /usr/include/eigen3/Eigen
)

# Add executable, basic node, uncalibrated
add_executable(bmi270_imu_node src/bmi270_imu_node.cpp)

# Add executable for the calibrated sensor
add_executable(bmi270_imu_calibtd_node src/bmi270_imu_calibtd_node.cpp)

# Add executable for the FIFO node
add_executable(bmi270_fifo_node src/bmi270_fifo_node.cpp) 


# Add executable for the FIFO attempt node
add_executable(bmi270_fifo_attempt src/bmi270_fifo_attempt.cpp) 

# Link libraries
# When linking to a library exported by another ament package with a namespace,
# use the full namespaced target name.
# The bmi270_driver exports 'bmi270_api' with the namespace 'bmi270_driver::'.
target_link_libraries(bmi270_imu_node
  bmi270_driver::bmi270_api # link using the namespaced target
  # i2c  # link with -li2c, doesn't seem required, maybe implicitly handled by ioctl/unistd headers
)
# Link libraries for the calibrated node
target_link_libraries(bmi270_imu_calibtd_node
  bmi270_driver::bmi270_api
  ${Eigen3_LIBS} # 
)

# Link libraries for the FIFO node
target_link_libraries(bmi270_fifo_node
  bmi270_driver::bmi270_api
)

# Link libraries for the FIFO attempt node
target_link_libraries(bmi270_fifo_attempt
  bmi270_driver::bmi270_api
)

# Link ROS2 dependencies using ament_target_dependencies
# This ensures that include paths and other necessary build flags are propagated

# for the basic node:
ament_target_dependencies(bmi270_imu_node
  rclcpp
  sensor_msgs
  bmi270_driver # This ensures the bmi270_driver package's exports are considered
)

# basic node with calibration:
ament_target_dependencies(bmi270_imu_calibtd_node
  rclcpp
  sensor_msgs
  bmi270_driver # This ensures the bmi270_driver package's exports are considered
  Eigen3
)
# Link ROS2 dependencies for the FIFO node
ament_target_dependencies(bmi270_fifo_node
  rclcpp
  sensor_msgs
  bmi270_driver
)

# Link ROS2 dependencies for the FIFO attempt
ament_target_dependencies(bmi270_fifo_attempt
  rclcpp
  sensor_msgs
  bmi270_driver
)

# Install executables
# simple polling mode, uncalibrated
install(TARGETS
  bmi270_imu_node
  DESTINATION lib/${PROJECT_NAME}
)
# polling mode, calibrated
install(TARGETS
  bmi270_imu_calibtd_node
  DESTINATION lib/${PROJECT_NAME}
)
# FIFO node executable
install(TARGETS
  bmi270_fifo_node
  DESTINATION lib/${PROJECT_NAME}
)

# FIFO node executable
install(TARGETS
  bmi270_fifo_attempt
  DESTINATION lib/${PROJECT_NAME}
)
# needed for ros2 run launch to find the launch file!
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

