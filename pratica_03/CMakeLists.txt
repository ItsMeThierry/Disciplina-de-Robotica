cmake_minimum_required(VERSION 3.8)
project(pratica_03)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pose_controller_node src/pose_controller_node.cpp)
ament_target_dependencies(pose_controller_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros control_toolbox)

add_executable(mission_follower_node src/mission_follower_node.cpp)
ament_target_dependencies(mission_follower_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(mission_follower_node yaml-cpp)

install(TARGETS pose_controller_node mission_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()