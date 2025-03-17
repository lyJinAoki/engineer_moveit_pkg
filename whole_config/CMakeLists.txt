cmake_minimum_required(VERSION 3.22)
project(whole_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

ament_package()

add_executable(multi_robot_planner src/multi_robot_planner.cpp)
ament_target_dependencies(multi_robot_planner 
    rclcpp geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface tf2 tf2_geometry_msgs control_msgs moveit_ros_move_group)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()
install(TARGETS multi_robot_planner DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY config src DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
