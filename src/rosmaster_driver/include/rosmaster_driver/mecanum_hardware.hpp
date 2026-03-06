//mecanum_hardware.hpp
#ifndef ROSMASTER_DRIVER__MECANUM_HARDWARE_HPP_
#define ROSMASTER_DRIVER__MECANUM_HARDWARE_HPP_

#include <array>
#include <string>
#include <vector>
#include <cstdint>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mecanum_hardware
{

class MecanumHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumHardware)

  // lifecycle-style initialization / activation hooks for Humble
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // regular read/write loop methods
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // store the hardware info provided on init
  hardware_interface::HardwareInfo info_;

  // serial communication
  int serial_fd_{-1};
  std::string serial_port_{"/dev/ttyUSB0"};
  int serial_baud_{115200};

  // wheel state & command arrays (4 wheels)
  std::array<double, 4> hw_wheel_velocity_state_{ {0.0, 0.0, 0.0, 0.0} };
  std::array<double, 4> hw_wheel_velocity_cmd_{ {0.0, 0.0, 0.0, 0.0} };

  // synchronization
  std::mutex hw_mutex_;

  // helper: format and send motor frame over serial
  // returns number of bytes written or -1 on error
  ssize_t send_motor_frame(int8_t s1, int8_t s2, int8_t s3, int8_t s4);

  // open/close helpers
  bool open_serial();
  void close_serial();

  rclcpp::Time last_update_time_;
};

}  // namespace mecanum_hardware

#endif  // ROSMASTER_DRIVER__MECANUM_HARDWARE_HPP_