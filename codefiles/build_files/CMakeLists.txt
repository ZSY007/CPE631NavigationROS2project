cmake_minimum_required(VERSION 3.8)
project(cpe631_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(cpe631_ros2_dstar_planner SHARED
  src/dstar_planner.cpp
)
target_include_directories(cpe631_ros2_dstar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cpe631_ros2_dstar_planner
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)
pluginlib_export_plugin_description_file(nav2_core cpe631_ros2_dstar_planner.xml)

install(DIRECTORY launch worlds models rviz param maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS cpe631_ros2_dstar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES cpe631_ros2_dstar_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  cpe631_ros2/teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cpe631_teleop
)

install(PROGRAMS
  cpe631_ros2/peds.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cpe631_peds
)

install(PROGRAMS
  cpe631_ros2/map_republisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cpe631_map_republisher
)

install(PROGRAMS
  cpe631_ros2/ped_pose_extractor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ped_pose_extractor
)

install(PROGRAMS
  cpe631_ros2/social_nav_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME social_nav_node
)

install(PROGRAMS
  cpe631_ros2/data_collector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME data_collector
)

install(PROGRAMS
  cpe631_ros2/goal_sender.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME goal_sender
)

install(PROGRAMS
  cpe631_ros2/replan_trigger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME replan_trigger
)

ament_export_include_directories(include)
ament_export_libraries(cpe631_ros2_dstar_planner)
ament_export_dependencies(
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

ament_package()
