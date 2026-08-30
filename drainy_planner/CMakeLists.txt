cmake_minimum_required(VERSION 3.20)
project(drainy_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_planner REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(drainy_common REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/drainy_planner/DrainyPlanner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_planner::easynav_planner
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ${tf2_geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${PCL_LIBRARIES}
  drainy_common::drainy_common
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register the planning plugins
pluginlib_export_plugin_description_file(easynav_core drainy_planner_plugins.xml)

ament_export_dependencies(
  easynav_common
  easynav_planner
  easynav_core
  tf2
  tf2_ros
  pluginlib
  nav_msgs
  drainy_common
)
ament_package()
