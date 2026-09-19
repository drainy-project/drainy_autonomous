cmake_minimum_required(VERSION 3.20)
project(drainy_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(easynav_system REQUIRED)
find_package(drainy_common REQUIRED)
find_package(easynav_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/drainy_explorer/DrainyExplorer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  easynav_system::easynav_system
  drainy_common::drainy_common
  ${easynav_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_executable(explorer src/drainy_explorer_main.cpp)

target_link_libraries(explorer PUBLIC
  ${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  explorer
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
endif()

ament_package()
