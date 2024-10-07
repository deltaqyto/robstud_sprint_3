cmake_minimum_required(VERSION 3.5)
project(robstud_sprint_3)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)


add_executable(censorship src/censorship.cpp)
ament_target_dependencies(censorship ${ament_index_cpp_LIBRARIES} rclcpp tf2 nav_msgs std_msgs sensor_msgs geometry_msgs OpenCV cv_bridge)
add_executable(map_server src/map_server.cpp)
ament_target_dependencies(map_server ${ament_index_cpp_LIBRARIES} rclcpp tf2 nav_msgs std_msgs sensor_msgs geometry_msgs OpenCV cv_bridge)

install(TARGETS
  censorship
  map_server
  DESTINATION lib/${PROJECT_NAME}/
  )

install(DIRECTORY
  launch
  data
  DESTINATION share/${PROJECT_NAME}/
  )
  
ament_package()