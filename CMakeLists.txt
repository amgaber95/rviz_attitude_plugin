cmake_minimum_required(VERSION 3.8)
project(rviz_attitude_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  Eigen3
  Qt5
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
