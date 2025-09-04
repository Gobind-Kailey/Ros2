cmake_minimum_required(VERSION 3.8)
project(ros2_course_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

# add the executable
add_executable(talker_node_cpp src/talker.cpp)

# link dependencies
ament_target_dependencies(talker_node_cpp
  rclcpp
  rclcpp_action
  example_interfaces
  std_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install target
install(TARGETS
  talker_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
