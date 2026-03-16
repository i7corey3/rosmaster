// rosmaster.hpp
// Header-only helper to build Rosmaster / Yahboom protocol frames
// Mirrors the Python Rosmaster class frame construction and checksum.
// Usage: include this header and call the build_* functions then send the returned vector<uint8_t> to serial.
//
// To compile quick-test:
//   g++ -std=c++17 -DROSMASTER_HELPER_TEST rosmaster_frames.hpp -o rosmaster_test && ./rosmaster_test
//



// std::cout << "Rosmaster frame helper demo\n";

// // Example 1: set_motor(50, -50, 0, 127)
// auto f1 = build_set_motor(50, -50, 0, 127);
// std::cout << "set_motor(50,-50,0,127): " << to_hex(f1) << "\n";
// // Expected (matches earlier Python example): FF FC 07 10 32 CE 00 7F 96

// // Example 2: set_beep(50)
// auto f2 = build_set_beep(50);
// std::cout << "set_beep(50): " << to_hex(f2) << "\n";
// // Example 3: set_car_run(forward=1, speed=50)
// auto f3 = build_set_car_run(1, 1, 50, false);
// std::cout << "set_car_run(forward,50): " << to_hex(f3) << "\n";

// // Example 4: set_car_motion(0.5, 0, 0)
// auto f4 = build_set_car_motion(0.5, 0.0, 0.0, 1);
// std::cout << "set_car_motion(0.5,0,0): " << to_hex(f4) << "\n";

// // Example 5: request encoders
// auto f5 = build_request_data(0x0D, 0x00);
// std::cout << "request encoders: " << to_hex(f5) << "\n";
// // Expected: FF FC 05 50 0D 00 62

// // Example 6: set_uart_servo(1,1500,500)
// auto f6 = build_set_uart_servo(1, 1500, 500);
// std::cout << "set_uart_servo(1,1500,500): " << to_hex(f6) << "\n";

// // Example 7: set_pid_param(0.5,0.1,0.3)
// auto f7 = build_set_pid_param(0.5, 0.1, 0.3, false);
// std::cout << "set_pid_param(0.5,0.1,0.3): " << to_hex(f7) << "\n";






#ifndef ROSMASTER_HPP_
#define ROSMASTER_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <array>
#include <optional>
#include <cstddef>
#include <cassert>

namespace rosmaster {

// Default device constants (same as your Python class)
constexpr uint8_t HEAD = 0xFF;
constexpr uint8_t DEVICE_ID_DEFAULT = 0xFC; // 252
constexpr int COMPLEMENT_FROM_DEVICE_ID(int device_id) { return 257 - device_id; }

inline std::optional<std::array<int32_t,4>> parse_encoder_frame(
  const std::vector<uint8_t> &frame,
  int expected_device_id = DEVICE_ID_DEFAULT)
{
  // minimal frame length: HEAD(1) + DEV-1(1) + ext_len(1) + ext_type(1) + payload(at least 1) + checksum(1)
  if (frame.size() < 6) return std::nullopt;
  // Check head
  if (frame[0] != HEAD) return std::nullopt;
  // second byte from device appears to be DEVICE_ID - 1 in the Python receiver
  if (frame[1] != static_cast<uint8_t>((expected_device_id - 1) & 0xFF)) return std::nullopt;

  uint8_t ext_len = frame[2];
  uint8_t ext_type = frame[3];

  // ext_len indicates the number of bytes after byte2? The Python receiver uses data_len = ext_len - 2
  // Here we expect: total_frame_len == 4 + data_len where data_len = ext_len - 2 (payload + checksum)
  if (ext_len < 2) return std::nullopt; // invalid
  std::size_t data_len = static_cast<std::size_t>(ext_len - 2); // includes payload bytes + checksum
  // expected total bytes after the first three bytes (HEAD, DEV-1, ext_len): ext_type + data_len
  // std::size_t expected_total = 3 + 1 + data_len; // indices: 0..expected_total inclusive?
  // total frame must be at least (HEAD + DEV-1 + ext_len + ext_type + payload + checksum)
  if (frame.size() < (4 + data_len)) return std::nullopt;

  // payload bytes are located at frame[4 .. 4+payload_len-1], checksum at frame[4+payload_len]
  if (data_len < 1) return std::nullopt; // must have at least checksum
  std::size_t payload_len = data_len - 1;
  const uint8_t *payload_ptr = frame.data() + 4;
  const uint8_t checksum_byte = frame[4 + payload_len];

  // compute checksum same as Python: check_sum = ext_len + ext_type + sum(payload_bytes)  (mod 256)
  unsigned int check_sum = 0;
  check_sum += static_cast<unsigned int>(ext_len);
  check_sum += static_cast<unsigned int>(ext_type);
  for (size_t i = 0; i < payload_len; ++i) {
    check_sum += static_cast<unsigned int>(payload_ptr[i]);
  }
  if ((check_sum & 0xFF) != checksum_byte) {
    // checksum mismatch
    return std::nullopt;
  }

  // We only care about encoder frames (FUNC_REPORT_ENCODER == 0x0D)
  constexpr uint8_t FUNC_REPORT_ENCODER = 0x0D;
  if (ext_type != FUNC_REPORT_ENCODER) return std::nullopt;

  // encoder payload expected to be 16 bytes (4 x int32 little-endian)
  if (payload_len < 16) return std::nullopt;

  // decode 4 little-endian signed 32-bit integers
  std::array<int32_t,4> enc{};
  for (size_t i = 0; i < 4; ++i) {
    const uint8_t *p = payload_ptr + i * 4;
    uint32_t u = (static_cast<uint32_t>(p[0])      ) |
                 (static_cast<uint32_t>(p[1]) <<  8) |
                 (static_cast<uint32_t>(p[2]) << 16) |
                 (static_cast<uint32_t>(p[3]) << 24);
    enc[i] = static_cast<int32_t>(u);
  }
  return enc;
}

// Helper: compute checksum exactly as in Python: sum(cmd, COMPLEMENT) & 0xff
inline uint8_t compute_checksum(const std::vector<uint8_t> &cmd, int device_id = DEVICE_ID_DEFAULT) {
  int complement = COMPLEMENT_FROM_DEVICE_ID(device_id);
  int s = complement;
  for (auto b : cmd) s += static_cast<int>(b);
  return static_cast<uint8_t>(s & 0xff);
}

// Helper: set length byte (3rd byte) to len(cmd)-1 then append checksum and return final frame.
inline std::vector<uint8_t> finalize_frame(std::vector<uint8_t> base, int device_id = DEVICE_ID_DEFAULT) {
  if (base.size() >= 3) {
    base[2] = static_cast<uint8_t>(base.size() - 1); // same rule used in Python
  }
  uint8_t chk = compute_checksum(base, device_id);
  base.push_back(chk);
  return base;
}

// Utility: little-endian pack int16 into two bytes
inline std::array<uint8_t,2> pack_i16_le(int16_t v) {
  uint16_t uv = static_cast<uint16_t>(v);
  return { static_cast<uint8_t>(uv & 0xFF), static_cast<uint8_t>((uv >> 8) & 0xFF) };
}

// Utility: little-endian pack int32 into four bytes
inline std::array<uint8_t,4> pack_i32_le(int32_t v) {
  uint32_t uv = static_cast<uint32_t>(v);
  return {
    static_cast<uint8_t>(uv & 0xFF),
    static_cast<uint8_t>((uv >> 8) & 0xFF),
    static_cast<uint8_t>((uv >> 16) & 0xFF),
    static_cast<uint8_t>((uv >> 24) & 0xFF)
  };
}

// Convert frame to printable hex string
inline std::string to_hex(const std::vector<uint8_t> &frame) {
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');
  for (size_t i=0;i<frame.size();++i) {
    oss << std::setw(2) << static_cast<int>(frame[i]);
    if (i+1 < frame.size()) oss << ' ';
  }
  return oss.str();
}

// ----------------- Frame builders (common commands) ------------------

// set_motor(speed1, speed2, speed3, speed4)
// speeds are signed bytes [-128..127]; Python clamps to [-100..100] with 127 special (keep)
inline std::vector<uint8_t> build_set_motor(int8_t s1, int8_t s2, int8_t s3, int8_t s4, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x10,
                               static_cast<uint8_t>(s1),
                               static_cast<uint8_t>(s2),
                               static_cast<uint8_t>(s3),
                               static_cast<uint8_t>(s4) };
  return finalize_frame(cmd, device_id);
}

// set_beep(on_time) where on_time packed as int16 little-endian
inline std::vector<uint8_t> build_set_beep(int16_t on_time, int device_id = DEVICE_ID_DEFAULT) {
  auto v = pack_i16_le(on_time);
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x05, 0x02, v[0], v[1] };
  return finalize_frame(cmd, device_id);
}

// set_pwm_servo(servo_id, angle) - servo_id (1..4), angle (0..180)
inline std::vector<uint8_t> build_set_pwm_servo(uint8_t servo_id, uint8_t angle, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x03, servo_id, angle };
  return finalize_frame(cmd, device_id);
}

// set_pwm_servo_all(a1,a2,a3,a4) - each angle 0..180 or 255 invalid
inline std::vector<uint8_t> build_set_pwm_servo_all(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x04, a1, a2, a3, a4 };
  return finalize_frame(cmd, device_id);
}

// set_colorful_lamps(led_id, r,g,b)
inline std::vector<uint8_t> build_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x05, led_id, r, g, b };
  return finalize_frame(cmd, device_id);
}

// set_colorful_effect(effect, speed=255, parm=255)
inline std::vector<uint8_t> build_set_rgb_effect(uint8_t effect, uint8_t speed=255, uint8_t parm=255, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x06, effect, speed, parm };
  return finalize_frame(cmd, device_id);
}

// set_car_run(state, speed, adjust=false)
// state: 0..7 ; speed int16 ; adjust toggles car_type high bit in python (not implemented here - send car_type separately)
inline std::vector<uint8_t> build_set_car_run(uint8_t car_type, uint8_t state, int16_t speed, bool adjust=false, int device_id = DEVICE_ID_DEFAULT) {
  uint8_t ct = car_type;
  if (adjust) ct = static_cast<uint8_t>(ct | 0x80);
  auto sp = pack_i16_le(speed);
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x11, ct, static_cast<uint8_t>(state), sp[0], sp[1] };
  return finalize_frame(cmd, device_id);
}

// set_car_motion(vx, vy, vz) - vx,vy,vz are floats in meters/sec; Python multiplies by 1000 and packs int16
inline std::vector<uint8_t> build_set_car_motion(double vx, double vy, double vz, uint8_t car_type = 0x01, int device_id = DEVICE_ID_DEFAULT) {
  int16_t ivx = static_cast<int16_t>(static_cast<int>(vx * 1000.0));
  int16_t ivy = static_cast<int16_t>(static_cast<int>(vy * 1000.0));
  int16_t ivz = static_cast<int16_t>(static_cast<int>(vz * 1000.0));
  auto bx = pack_i16_le(ivx);
  auto by = pack_i16_le(ivy);
  auto bz = pack_i16_le(ivz);
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x12, car_type,
                               bx[0], bx[1], by[0], by[1], bz[0], bz[1] };
  return finalize_frame(cmd, device_id);
}

// set_uart_servo(servo_id, pulse_value, run_time=500)
// pulse_value and run_time are int16 little-endian
inline std::vector<uint8_t> build_set_uart_servo(uint8_t servo_id, int16_t pulse_value, int16_t run_time = 500, int device_id = DEVICE_ID_DEFAULT) {
  auto pv = pack_i16_le(pulse_value);
  auto rt = pack_i16_le(run_time);
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x20, servo_id, pv[0], pv[1], rt[0], rt[1] };
  return finalize_frame(cmd, device_id);
}

// set_uart_servo_angle(s_id, s_angle, run_time=500)
// Note: Python converts angle to pulse via __arm_convert_value; header doesn't implement conversion logic.
// If you have conversion mapping, call that first and use build_set_uart_servo.
inline std::vector<uint8_t> build_set_uart_servo_id(uint8_t new_id, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x04, 0x21, new_id };
  return finalize_frame(cmd, device_id);
}

// set_uart_servo_torque(enable)
inline std::vector<uint8_t> build_set_uart_servo_torque(bool enable, int device_id = DEVICE_ID_DEFAULT) {
  uint8_t on = enable ? 1 : 0;
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x04, 0x22, on };
  return finalize_frame(cmd, device_id);
}

// set_arm_ctrl (angle array + run_time) - expects pre-converted pulse values for each servo (6 * int16)
inline std::vector<uint8_t> build_set_arm_ctrl(const std::array<int16_t,6> &pulse_vals, int16_t run_time = 500, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x00, 0x23 };
  for (int i=0;i<6;++i) {
    auto p = pack_i16_le(pulse_vals[i]);
    cmd.push_back(p[0]); cmd.push_back(p[1]);
  }
  auto rt = pack_i16_le(run_time);
  cmd.push_back(rt[0]); cmd.push_back(rt[1]);
  return finalize_frame(cmd, device_id);
}

// set_pid_param(kp, ki, kd, forever=false)
// kp,ki,kd are floats; multiplied by 1000 and sent as int16, then state byte 0x5F if forever
inline std::vector<uint8_t> build_set_pid_param(double kp, double ki, double kd, bool forever=false, int device_id = DEVICE_ID_DEFAULT) {
  int16_t ikp = static_cast<int16_t>(kp * 1000.0);
  int16_t iki = static_cast<int16_t>(ki * 1000.0);
  int16_t ikd = static_cast<int16_t>(kd * 1000.0);
  auto ap = pack_i16_le(ikp);
  auto bp = pack_i16_le(iki);
  auto cp = pack_i16_le(ikd);
  uint8_t state = forever ? 0x5F : 0x00;
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x0A, 0x13,
                               ap[0], ap[1], bp[0], bp[1], cp[0], cp[1], state };
  return finalize_frame(cmd, device_id);
}

// request_data(function, param=0)
// This matches Python's __request_data: [HEAD, DEVICE_ID, 0x05, FUNC_REQUEST_DATA, function, param, checksum]
inline std::vector<uint8_t> build_request_data(uint8_t function, uint8_t param=0, int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x05, 0x50, function, param };
  return finalize_frame(cmd, device_id);
}

// reset flash
inline std::vector<uint8_t> build_reset_flash(int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x04, 0xA0, 0x5F };
  return finalize_frame(cmd, device_id);
}

// reset car state
inline std::vector<uint8_t> build_reset_car_state(int device_id = DEVICE_ID_DEFAULT) {
  std::vector<uint8_t> cmd = { HEAD, static_cast<uint8_t>(device_id), 0x04, 0x0F, 0x5F };
  return finalize_frame(cmd, device_id);
}

} // namespace rosmaster



#endif // ROSMASTER_HPP_