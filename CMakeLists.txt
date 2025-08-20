cmake_minimum_required(VERSION 3.8)
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

# Header-only utility files (no .cpp needed)
set(UTILITY_HEADERS
  include/rviz_attitude_plugin/euler_converter.hpp
  include/rviz_attitude_plugin/topic_utilities.hpp
  include/rviz_attitude_plugin/supported_types.hpp
)

# Widget sources (consolidated)
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

# Overlay system (Ogre integration)
set(OVERLAY_SOURCES
  src/overlay_system.cpp
)

set(OVERLAY_HEADERS
  include/rviz_attitude_plugin/overlay_system.hpp
)

# Main display class
set(DISPLAY_SOURCES
  src/attitude_display.cpp
)

set(DISPLAY_HEADERS
  include/rviz_attitude_plugin/attitude_display.hpp
)

# Combine all sources and headers
set(ALL_SOURCES
  ${WIDGET_SOURCES}
  ${OVERLAY_SOURCES}
  ${DISPLAY_SOURCES}
)

set(ALL_HEADERS
  ${WIDGET_HEADERS}
  ${OVERLAY_HEADERS}
  ${DISPLAY_HEADERS}
  ${UTILITY_HEADERS}
)

# Create the plugin library
add_library(${PROJECT_NAME} SHARED
  ${ALL_SOURCES}
  ${ALL_HEADERS}
)

# Add ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Link Qt5 libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

# Plugin registration
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py"
  )
endif()

# Install config files (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.rviz"
  )
endif()

# Install scripts (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/sample_imu_publisher.py)
  install(PROGRAMS
    scripts/sample_imu_publisher.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_export_dependencies(
  ${dependencies}
  Qt5
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
