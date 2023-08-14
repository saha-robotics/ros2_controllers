cmake_minimum_required(VERSION 3.16)
project(battery_info_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(battery_info_broadcaster_parameters
  src/battery_info_broadcaster_parameters.yaml
)

add_library(battery_info_broadcaster SHARED
  src/battery_info_broadcaster.cpp
)
target_compile_features(battery_info_broadcaster PUBLIC cxx_std_17)
target_include_directories(battery_info_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/battery_info_broadcaster>
)
target_link_libraries(battery_info_broadcaster PUBLIC
  battery_info_broadcaster_parameters
)
ament_target_dependencies(battery_info_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(battery_info_broadcaster PRIVATE "BATTERY_INFO_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface battery_info_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/battery_info_broadcaster
)
install(
  TARGETS
    battery_info_broadcaster
    battery_info_broadcaster_parameters
  EXPORT export_battery_info_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_battery_info_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
