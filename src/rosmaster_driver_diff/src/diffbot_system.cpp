// diffbot_system.cpp
#include "rosmaster_driver_diff/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Ensure you include the controller header
#include "rosmaster_driver_diff/rosmaster_controller.hpp"

namespace diffdrive_rosmaster
{

// on_init
DiffDriveRosmasterHardware::CallbackReturn
DiffDriveRosmasterHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      DiffDriveRosmasterHardware::CallbackReturn::SUCCESS)
  {
    return DiffDriveRosmasterHardware::CallbackReturn::ERROR;
  }

  // store reference time
  time_ = std::chrono::system_clock::now();

  // Read hardware parameters
  try {
    cfg_.front_left_wheel_name  = info_.hardware_parameters.at("front_left_wheel_name");
    cfg_.front_right_wheel_name = info_.hardware_parameters.at("front_right_wheel_name");
    cfg_.back_left_wheel_name   = info_.hardware_parameters.at("back_left_wheel_name");
    cfg_.back_right_wheel_name  = info_.hardware_parameters.at("back_right_wheel_name");
    cfg_.loop_rate              = std::stof(info_.hardware_parameters.at("loop_rate"));
    cfg_.device                 = info_.hardware_parameters.at("device");
    cfg_.baud_rate              = std::stoi(info_.hardware_parameters.at("baud_rate"));
    cfg_.timeout                = std::stoi(info_.hardware_parameters.at("timeout"));
    cfg_.enc_counts_per_rev     = std::stoi(info_.hardware_parameters.at("enc_counts_per_rev"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRosmasterHardware"),
                 "Failed to parse hardware parameters: %s", e.what());
    return DiffDriveRosmasterHardware::CallbackReturn::ERROR;
  }

  // Configure wheel helper objects (assume Wheel::setup exists)
  fl_wheel_.setup(cfg_.front_left_wheel_name,  cfg_.enc_counts_per_rev);
  fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  bl_wheel_.setup(cfg_.back_left_wheel_name,   cfg_.enc_counts_per_rev);
  br_wheel_.setup(cfg_.back_right_wheel_name,  cfg_.enc_counts_per_rev);

  // Initialize controller / serial
  if (!controller_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout)) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRosmasterHardware"),
                 "Failed to initialize controller");
    return DiffDriveRosmasterHardware::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRosmasterHardware"),
              "Finished configuration (device=%s, baud=%d, timeout=%d)",
              cfg_.device.c_str(), cfg_.baud_rate, cfg_.timeout);

  return DiffDriveRosmasterHardware::CallbackReturn::SUCCESS;
}

// export_state_interfaces
std::vector<hardware_interface::StateInterface>
DiffDriveRosmasterHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_POSITION, &bl_wheel_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_POSITION, &br_wheel_.pos));

  return state_interfaces;
}

// export_command_interfaces
std::vector<hardware_interface::CommandInterface>
DiffDriveRosmasterHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.cmd));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.cmd));

  return command_interfaces;
}

// on_activate
DiffDriveRosmasterHardware::CallbackReturn
DiffDriveRosmasterHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRosmasterHardware"),
              "Activating hardware interface...");
  return DiffDriveRosmasterHardware::CallbackReturn::SUCCESS;
}

// on_deactivate
DiffDriveRosmasterHardware::CallbackReturn
DiffDriveRosmasterHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRosmasterHardware"),
              "Deactivating hardware interface...");
  controller_.Estop();
  return DiffDriveRosmasterHardware::CallbackReturn::SUCCESS;
}

// read
hardware_interface::return_type
DiffDriveRosmasterHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double deltaSeconds = period.seconds();
  if (deltaSeconds <= 0.0) {
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    deltaSeconds = diff.count();
    time_ = new_time;
  }

  if (!controller_.connected()) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveRosmasterHardware"),
                "Controller not connected on read()");
    return hardware_interface::return_type::ERROR;
  }

  std::array<int32_t,4> enc_cache{0,0,0,0};
  controller_.getLatestCachedEnc(enc_cache);
  fl_wheel_.enc = static_cast<long>(enc_cache[0]);
  fr_wheel_.enc = static_cast<long>(enc_cache[2]);

  double prev_pos = fl_wheel_.pos;
  fl_wheel_.pos = fl_wheel_.calcEncAngle(fl_wheel_.enc_offset);
  fl_wheel_.vel = (fl_wheel_.pos - prev_pos) / deltaSeconds;

  prev_pos = fr_wheel_.pos;
  fr_wheel_.pos = fr_wheel_.calcEncAngle(fr_wheel_.enc_offset);
  fr_wheel_.vel = (fr_wheel_.pos - prev_pos) / deltaSeconds;

  bl_wheel_.pos = fl_wheel_.pos;
  br_wheel_.pos = fr_wheel_.pos;
  bl_wheel_.vel = fl_wheel_.vel;
  br_wheel_.vel = fr_wheel_.vel;

  return hardware_interface::return_type::OK;
}

// write
hardware_interface::return_type
DiffDriveRosmasterHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!controller_.connected()) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveRosmasterHardware"),
                "Controller not connected on write()");
    return hardware_interface::return_type::ERROR;
  }

  // Safety E-stop sentinel
  if (std::abs(static_cast<long>(fl_wheel_.cmd)) >= 20000L ||
      std::abs(static_cast<long>(fr_wheel_.cmd)) >= 20000L) {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveRosmasterHardware"),
                "E-STOP triggered, command too large");
    controller_.Estop();
    return hardware_interface::return_type::OK;
  }

  // Use the topic value directly (assumed to be desired velocity value).
  double cmd_left_d  = static_cast<double>(fl_wheel_.cmd);
  double cmd_right_d = static_cast<double>(fr_wheel_.cmd);

  // Round to integer motor units (device expects ints)
  long pos_1 = static_cast<long>(std::lround(cmd_left_d));
  long pos_2 = static_cast<long>(std::lround(cmd_right_d));

  // Clamp to protocol (-100..100) preserving 127 sentinel
  auto clamp_to_protocol = [](long v)->long {
    if (v == 127L) return 127L;
    if (v > 100L) return 100L;
    if (v < -100L) return -100L;
    return v;
  };

  long send_left  = clamp_to_protocol(pos_1);
  long send_right = clamp_to_protocol(pos_2);

  // // Log velocity (topic value) and actual sent values
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveRosmasterHardware"),
  //             "Motor write | topic_vel L=%.6f R=%.6f | rounded L=%ld R=%ld | sent L=%ld R=%ld",
  //             cmd_left_d, cmd_right_d, pos_1, pos_2, send_left, send_right);

  controller_.setMotorValues(send_left, send_right);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_rosmaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_rosmaster::DiffDriveRosmasterHardware, hardware_interface::SystemInterface)