// src/pluginlib_export.cpp
#include <pluginlib/class_list_macros.hpp>
#include "rosmaster_driver/mecanum_hardware.hpp"

// The first argument MUST match the xml <class type="..."> value
// and the actual namespace/class in your code.
PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumHardware, hardware_interface::SystemInterface)