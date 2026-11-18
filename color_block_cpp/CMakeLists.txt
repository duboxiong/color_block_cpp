cmake_minimum_required(VERSION 3.8)
project(color_block_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTargetColor.srv"
)

add_executable(vision_server src/vision_server.cpp)
ament_target_dependencies(vision_server rclcpp sensor_msgs cv_bridge OpenCV foxglove_msgs)
rosidl_get_typesupport_target(vision_server_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vision_server "${vision_server_cpp}")

add_executable(color_client src/color_client.cpp)
ament_target_dependencies(color_client rclcpp)
rosidl_get_typesupport_target(color_client_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(color_client "${color_client_cpp}")

install(TARGETS
  vision_server
  color_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
