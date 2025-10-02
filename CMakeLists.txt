cmake_minimum_required(VERSION 3.5)
project(rviz_attitude_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
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
find_package(tf2_geometry_msgs REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
set(CMAKE_AUTOMOC ON)

# Include directories
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Source files for widgets
set(WIDGET_SOURCES
  src/widgets/attitude_indicator.cpp
  src/widgets/heading_indicator.cpp
  src/widgets/angle_readout.cpp
)

set(WIDGET_HEADERS
  include/rviz_attitude_plugin/widgets/attitude_indicator.hpp
  include/rviz_attitude_plugin/widgets/heading_indicator.hpp
  include/rviz_attitude_plugin/widgets/angle_readout.hpp
)

# Header-only utility files (no .cpp needed)
set(UTILITY_HEADERS
  include/rviz_attitude_plugin/euler_converter.hpp
  include/rviz_attitude_plugin/topic_utilities.hpp
)

# Main plugin sources
set(PLUGIN_SOURCES
  src/attitude_display.cpp
  src/attitude_widget.cpp
  src/overlay_system.cpp
)

set(PLUGIN_HEADERS
  include/rviz_attitude_plugin/attitude_display.hpp
  include/rviz_attitude_plugin/attitude_widget.hpp
  include/rviz_attitude_plugin/overlay_system.hpp
)

# Build the plugin library
add_library(${PROJECT_NAME} SHARED
  ${PLUGIN_SOURCES}
  ${WIDGET_SOURCES}
  ${PLUGIN_HEADERS}
  ${WIDGET_HEADERS}
  ${UTILITY_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Register plugin description with pluginlib (ROS 2 ament index)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)


# Export
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
