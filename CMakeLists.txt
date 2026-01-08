cmake_minimum_required(VERSION 3.8)
project(group18_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

# generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  "srv/GripperRequest.srv"
  DEPENDENCIES geometry_msgs action_msgs unique_identifier_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

add_executable(motion_planner src/motion_planner.cpp)
ament_target_dependencies(motion_planner
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)   
target_link_libraries(motion_planner "${cpp_typesupport_target}")

add_executable(gripper_node
  src/gripper_node.cpp
)
target_include_directories(gripper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gripper_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
)
target_link_libraries(gripper_node "${cpp_typesupport_target}")

add_executable(swap_coordinator src/swap_coordinator.cpp)
ament_target_dependencies(swap_coordinator
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)   
target_link_libraries(swap_coordinator "${cpp_typesupport_target}") 

add_executable(collision_detector src/collision_detector.cpp)
ament_target_dependencies(collision_detector
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  geometry_msgs
)   
target_link_libraries(collision_detector "${cpp_typesupport_target}")


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

install(TARGETS
  motion_planner
  gripper_node
  swap_coordinator
  collision_detector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
