cmake_minimum_required(VERSION 3.16)
project(rosmaster_driver_diff LANGUAGES CXX)

# Default to C++17 (was mixed C++14 / cxx_std_17; make consistent)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  diffdrive_rosmaster
  SHARED
  src/diffbot_system.cpp
)

# set C++ standard as target property (consistent with above)
target_compile_features(diffdrive_rosmaster PUBLIC cxx_std_17)

target_include_directories(diffdrive_rosmaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diffdrive_rosmaster>
)

ament_target_dependencies(
  diffdrive_rosmaster PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# If you link against the system 'serial' lib, keep it; adjust if different
target_link_libraries(diffdrive_rosmaster PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# Use the actual target name here (not ${PROJECT_NAME})
target_compile_definitions(diffdrive_rosmaster PRIVATE "diffdrive_rosmaster_BUILDING_DLL")

# Export hardware plugins (registers plugin xml so ament index / pluginlib can find it)
pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/diffdrive_rosmaster
)

# Install the plugin XML so pluginlib can discover it at runtime
install(FILES
  robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS diffdrive_rosmaster
  EXPORT export_diffdrive_rosmaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_rosmaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()