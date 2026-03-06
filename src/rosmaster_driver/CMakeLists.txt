# cmake_minimum_required(VERSION 3.5)
# project(rosmaster_driver)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# find_package(ros2_control_cmake REQUIRED)
# find_package(controller_manager 4.0.0)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(ament_index_cpp REQUIRED)
# find_package(Boost REQUIRED)
# find_package(Threads REQUIRED)

# add_library(mecanum_hardware SHARED
#   src/mecanum_hardware.cpp
#   # remove this if you don't have pluginlib_export.cpp
#   # src/pluginlib_export.cpp
# )

# target_include_directories(mecanum_hardware PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(mecanum_hardware
#   rclcpp
#   rclcpp_lifecycle
#   hardware_interface
#   controller_manager
#   pluginlib
#   ament_index_cpp
#   Boost
# )

# # use plain form to stay consistent with ament_target_dependencies
# target_link_libraries(mecanum_hardware
#   Threads::Threads
# )

# install(
#   TARGETS mecanum_hardware
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/ DESTINATION include)
# install(FILES mecanum_hardware.xml DESTINATION share/${PROJECT_NAME})

# ament_export_libraries(mecanum_hardware)
# ament_package()