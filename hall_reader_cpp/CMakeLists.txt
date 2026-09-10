cmake_minimum_required(VERSION 3.8)
project(hall_reader_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(arduino_odom_node src/arduino_odom_node.cpp)
ament_target_dependencies(arduino_odom_node rclcpp nav_msgs geometry_msgs sensor_msgs tf2 tf2_ros)

# Install executable
install(TARGETS arduino_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY

  DESTINATION share/${PROJECT_NAME}
)

ament_package()

