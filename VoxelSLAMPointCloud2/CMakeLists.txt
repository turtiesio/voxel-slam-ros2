cmake_minimum_required(VERSION 3.8)
project(voxelslam_pointcloud2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Skip build if rviz_common is not available (headless environment)
find_package(rviz_common QUIET)
if(NOT rviz_common_FOUND)
  message(STATUS "rviz_common not found - skipping ${PROJECT_NAME} (RViz plugin)")
  ament_package()
  return()
endif()

set(CMAKE_AUTOMOC ON)

find_package(rclcpp REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(SRC_FILES
  src/voxelslam_pc2.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  sensor_msgs
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME})
ament_package()
