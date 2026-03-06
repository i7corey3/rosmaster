// mecanum_hardware.cpp
#include "rosmaster_driver/mecanum_hardware.hpp"

#include <algorithm>
#include <numeric>
#include <unistd.h>     // ::write, ::close
#include <fcntl.h>      // open flags
#include <termios.h>    // termios, cfmakeraw
#include <sys/types.h>
#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace mecanum_hardware
{

/* ---- Constants matching the Python driver ---- */
static constexpr uint8_t HEAD = 0xFF;
static constexpr uint8_t DEVICE_ID = 0xFC;
static constexpr uint16_t COMPLEMENT = 257 - DEVICE_ID;
static constexpr uint8_t FUNC_MOTOR = 0x10;

/* on_init: store info and read optional hardware parameters */
hardware_interface::CallbackReturn MecanumHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  // defaults
  serial_port_ = "/dev/ttyUSB0";
  serial_baud_ = 115200;
  serial_fd_ = -1;

  // read serial params from hardware parameters (if provided)
  for (const auto &p : info_.hardware_parameters) {
    if (p.first == "serial_port") {
      serial_port_ = p.second;
    } else if (p.first == "baud") {
      try {
        serial_baud_ = std::stoi(p.second);
      } catch (...) { /* keep default */ }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),
              "on_init() serial_port=%s baud=%d",
              serial_port_.c_str(), serial_baud_);

  // initialize state & command arrays
  hw_wheel_velocity_state_.fill(0.0);
  hw_wheel_velocity_cmd_.fill(0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* on_activate: open serial and prepare */
hardware_interface::CallbackReturn MecanumHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Activating Mecanum hardware");

  if (!open_serial()) {
    RCLCPP_ERROR(rclcpp::get_logger("MecanumHardware"),
                 "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // clear commands and states
  {
    std::lock_guard<std::mutex> lock(hw_mutex_);
    hw_wheel_velocity_cmd_.fill(0.0);
    hw_wheel_velocity_state_.fill(0.0);
  }

  last_update_time_ = rclcpp::Clock().now();
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* on_deactivate: close serial */
hardware_interface::CallbackReturn MecanumHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Deactivating Mecanum hardware");
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* Export state interfaces (velocity for each wheel) */
std::vector<hardware_interface::StateInterface> MecanumHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(4);

  for (size_t i = 0; i < 4; ++i) {
    // expect info_.joints.size() == 4; guard for safety
    std::string joint_name = (i < info_.joints.size()) ? info_.joints[i].name : ("wheel_" + std::to_string(i+1));
    interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_velocity_state_[i]));
  }
  return interfaces;
}

/* Export command interfaces (velocity command for each wheel) */
std::vector<hardware_interface::CommandInterface> MecanumHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(4);

  for (size_t i = 0; i < 4; ++i) {
    std::string joint_name = (i < info_.joints.size()) ? info_.joints[i].name : ("wheel_" + std::to_string(i+1));
    interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_velocity_cmd_[i]));
  }
  return interfaces;
}

/* read: update state variables from hardware (placeholder) */
hardware_interface::return_type MecanumHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // If your hardware reports encoder/velocity back, read it here and update hw_wheel_velocity_state_.
  // This template doesn't parse incoming frames (you can add it later).
  std::lock_guard<std::mutex> lock(hw_mutex_);
  // currently no incoming parsing — assume last commanded velocity reached
  for (size_t i = 0; i < 4; ++i) {
    hw_wheel_velocity_state_[i] = hw_wheel_velocity_cmd_[i];
  }
  return hardware_interface::return_type::OK;
}

/* write: send wheel commands to the hardware */
hardware_interface::return_type MecanumHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // Convert commanded velocities into -100..100 int8_t motor values.
  int8_t s1, s2, s3, s4;
  {
    std::lock_guard<std::mutex> lock(hw_mutex_);
    auto clamp_to_int8 = [](double v) -> int8_t {
      // user may send meters/sec or rad/sec; this example assumes the controller uses a
      // scale that maps directly to [-100, 100]. Adjust mapping as needed.
      double clamped = std::max(-100.0, std::min(100.0, v));
      return static_cast<int8_t>(std::round(clamped));
    };
    s1 = clamp_to_int8(hw_wheel_velocity_cmd_[0]);
    s2 = clamp_to_int8(hw_wheel_velocity_cmd_[1]);
    s3 = clamp_to_int8(hw_wheel_velocity_cmd_[2]);
    s4 = clamp_to_int8(hw_wheel_velocity_cmd_[3]);
  }

  ssize_t sent = send_motor_frame(s1, s2, s3, s4);
  if (sent < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MecanumHardware"), "Failed to send motor frame");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

/* open_serial: open and configure a POSIX serial port */
bool MecanumHardware::open_serial()
{
  if (serial_fd_ >= 0) {
    return true;  // already open
  }

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Could not open serial port %s: %s",
                serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  // configure termios
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "tcgetattr failed: %s", std::strerror(errno));
    ::close(serial_fd_); serial_fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);

  // set baud rate
  speed_t baud;
  switch (serial_baud_) {
    case 115200: baud = B115200; break;
    case 57600: baud = B57600; break;
    case 38400: baud = B38400; break;
    case 19200: baud = B19200; break;
    default: baud = B115200; break;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5; // 0.5s read timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "tcsetattr failed: %s", std::strerror(errno));
    ::close(serial_fd_); serial_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Serial opened: %s @ %d", serial_port_.c_str(), serial_baud_);
  return true;
}

/* close_serial */
void MecanumHardware::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Serial closed");
  }
}

/* send_motor_frame: create frame like Python Rosmaster.set_motor and write it */
ssize_t MecanumHardware::send_motor_frame(int8_t s1, int8_t s2, int8_t s3, int8_t s4)
{
  if (serial_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Serial not open, cannot send frame");
    return -1;
  }

  std::vector<uint8_t> cmd;
  // HEAD, DEVICE_ID, length placeholder, FUNC_MOTOR, speeds (signed bytes)
  cmd.push_back(HEAD);
  cmd.push_back(DEVICE_ID);
  cmd.push_back(0x00); // length placeholder, will set later
  cmd.push_back(FUNC_MOTOR);
  // pack the speeds as signed bytes (same as struct.pack('b') in python)
  cmd.push_back(static_cast<uint8_t>(static_cast<int8_t>(s1)));
  cmd.push_back(static_cast<uint8_t>(static_cast<int8_t>(s2)));
  cmd.push_back(static_cast<uint8_t>(static_cast<int8_t>(s3)));
  cmd.push_back(static_cast<uint8_t>(static_cast<int8_t>(s4)));

  // set length: total_len - 1 (same as python: cmd[2] = len(cmd) - 1)
  cmd[2] = static_cast<uint8_t>(cmd.size() - 1);

  // checksum: sum(cmd, COMPLEMENT) & 0xff like Python
  uint32_t acc = static_cast<uint32_t>(COMPLEMENT);
  for (uint8_t b : cmd) acc += b;
  uint8_t checksum = static_cast<uint8_t>(acc & 0xff);
  cmd.push_back(checksum);

  // write via POSIX write; use ::write to avoid clash with member write()
  ssize_t written = ::write(serial_fd_, reinterpret_cast<const void *>(cmd.data()), cmd.size());
  if (written < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MecanumHardware"), "Serial write error: %s", std::strerror(errno));
    return -1;
  }

  if (written != static_cast<ssize_t>(cmd.size())) {
    RCLCPP_WARN(rclcpp::get_logger("MecanumHardware"), "Partial serial write: %zd of %zu", written, cmd.size());
  }

  // debug print of raw frame (as hex) using RCLCPP_DEBUG
  {
    std::ostringstream ss;
    ss << std::hex << std::setfill('0');
    for (auto b : cmd) {
      ss << std::setw(2) << static_cast<int>(b) << ' ';
    }
    RCLCPP_DEBUG(rclcpp::get_logger("MecanumHardware"), "motor frame: %s", ss.str().c_str());
  }

  return written;
}

}  // namespace mecanum_hardware

// plugin export
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumHardware, hardware_interface::SystemInterface)