cmake_minimum_required(VERSION 3.16)
project(standart_message_broadcaster LANGUAGES CXX)

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

generate_parameter_library(standart_message_broadcaster_parameters
  src/standart_message_broadcaster_parameters.yaml
)

add_library(standart_message_broadcaster SHARED
  src/standart_message_broadcaster.cpp
)
target_compile_features(standart_message_broadcaster PUBLIC cxx_std_17)
target_include_directories(standart_message_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/standart_message_broadcaster>
)
target_link_libraries(standart_message_broadcaster PUBLIC
  standart_message_broadcaster_parameters
)
ament_target_dependencies(standart_message_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(standart_message_broadcaster PRIVATE "STANDART_MESSAGE_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface standart_message_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/standart_message_broadcaster
)
install(
  TARGETS
    standart_message_broadcaster
    standart_message_broadcaster_parameters
  EXPORT export_standart_message_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_standart_message_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
