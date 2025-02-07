cmake_minimum_required(VERSION 3.22)
project(bot_movit)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# Generate interfaces before `ament_package()`
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetJointP.srv"
  DEPENDENCIES std_msgs
)

# Create the service executable
add_executable(move_bot_service src/move_bot_service.cpp)
target_include_directories(move_bot_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  PROGRAMS scripts/qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_target_dependencies(move_bot_service
  rclcpp
  moveit_ros_planning_interface
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(move_bot_service "${cpp_typesupport_target}")

install(TARGETS move_bot_service
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Ensure `ament_package()` is the last statement
ament_package()
