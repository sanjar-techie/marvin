cmake_minimum_required(VERSION 3.8)
project(motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(motor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  ${ament_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_executable(md_robot_node src/main3.cpp src/com.cpp src/robot.cpp)
# add_dependencies(md_robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} rclcpp std_msgs geometry_msgs)
target_link_libraries(md_robot_node serial)
ament_target_dependencies(md_robot_node rclcpp geometry_msgs motor_msgs)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# install(TARGETS md_robot_node
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION bin/${PROJECT_NAME})

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include
#   FILES_MATCHING PATTERN "*.h"
# )

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(rclpy)
ament_export_include_directories(include)
ament_export_libraries(md)

ament_package()
