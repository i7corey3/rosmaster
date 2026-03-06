#ifndef ROSMASTER_DRIVER__DIFFBOT_SYSTEM_HPP_
#define ROSMASTER_DRIVER__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rosmaster_driver_diff/visibility_control.h"

#include "rosmaster_driver_diff/rosmaster_controller.hpp"
#include "rosmaster_driver_diff/wheel.hpp"

namespace diffdrive_rosmaster
{

class DiffDriveRosmasterHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveRosmasterHardware);

  // Alias for lifecycle callback return type
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  struct Config
  {
    std::string front_left_wheel_name = "";
    std::string front_right_wheel_name = "";
    std::string back_left_wheel_name = "";
    std::string back_right_wheel_name = "";
    float loop_rate = 0;
    std::string device = "";
    int baud_rate = 0;
    int timeout = 0;
    int enc_counts_per_rev = 0;
  };

  DIFFDRIVE_ROSMASTER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_ROSMASTER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_ROSMASTER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_ROSMASTER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ROSMASTER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // NOTE: Humble SystemInterface requires read/write to accept time & period
  DIFFDRIVE_ROSMASTER_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_ROSMASTER_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  RosmasterController controller_;
  Config cfg_;
  Wheel fl_wheel_;
  Wheel fr_wheel_;
  Wheel bl_wheel_;
  Wheel br_wheel_;

  std::chrono::time_point<std::chrono::system_clock> time_;
};

}  // namespace diffdrive_rosmaster

#endif  // ROSMASTER_DRIVER__DIFFBOT_SYSTEM_HPP_