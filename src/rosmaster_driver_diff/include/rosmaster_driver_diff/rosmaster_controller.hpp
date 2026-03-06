// rosmaster_controller.hpp
// RosmasterController: async writer + prioritized latest-motor-slot + low-latency reader
// with cached encoder values for non-blocking reads.
// Drop into include/rosmaster_driver_diff/rosmaster_controller.hpp

#ifndef ROSMASTER_CONTROLLER_HPP
#define ROSMASTER_CONTROLLER_HPP

#include <sstream>
#include <iostream>
#include <deque>
#include <array>
#include <optional>
#include <vector>
#include <mutex>
#include <chrono>
#include <thread>
#include <algorithm>
#include <atomic>
#include <stdexcept>
#include <condition_variable>
#include <iomanip>

#include <libserial/SerialPort.h>   // may be <LibSerial/SerialPort.h> depending on install
#include "rclcpp/rclcpp.hpp"

#include "rosmaster.hpp" // frame builder + parser (must provide build_set_motor, build_request_data, parse_encoder_frame, etc.)

using namespace rosmaster;

class RosmasterController
{
public:
  RosmasterController() = default;

  ~RosmasterController() {
    // stop reader thread
    running_.store(false);
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    // stop writer thread
    writer_running_.store(false);
    write_cv_.notify_all();
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }

    // close serial safely
    std::lock_guard<std::mutex> lk(write_mtx_);
    if (serial_conn_.IsOpen()) {
      try { serial_conn_.Close(); }
      catch (...) { /* swallow */ }
    }
  }

  // returns true on success
  bool setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    try {
      serial_conn_.Open(serial_device);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("RosmasterController"),
                   "Failed to open serial device %s: %s", serial_device.c_str(), e.what());
      return false;
    }

    try {
      serial_conn_.SetBaudRate(convert_baud_rate(static_cast<int>(baud_rate)));
      serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("RosmasterController"),
                   "Failed to configure serial device %s: %s", serial_device.c_str(), e.what());
      try { serial_conn_.Close(); } catch (...) {}
      return false;
    }

    // Initial safety stop (blocking write to ensure motors are off)
    {
      std::lock_guard<std::mutex> lk(write_mtx_);
      WriteToDriverUnlocked(build_set_motor(0, 0, 0, 0));
      WriteToDriverUnlocked(build_set_rgb_effect(0));
    }

    // start reader thread AFTER serial is ready
    running_.store(true);
    reader_thread_ = std::thread(&RosmasterController::readerLoop, this);

    // start writer thread AFTER serial is ready
    writer_running_.store(true);
    writer_thread_ = std::thread(&RosmasterController::writerLoop, this);

    RCLCPP_INFO(rclcpp::get_logger("RosmasterController"),
                "RosmasterController setup complete (device=%s, timeout=%d)",
                serial_device.c_str(), timeout_ms);

    return true;
  }

  // readEncoderValues wrapper keeps original signature (blocking poll)
  void readEncoderValues(long &val_1, long &val_2)
  {
    std::array<int32_t,4> enc;
    if (getEncoderValues(enc, static_cast<unsigned int>(timeout_ms_))) {
      val_1 = static_cast<long>(enc[0]);
      val_2 = static_cast<long>(enc[2]);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("RosmasterController"), "Failed to read binary encoders (timeout)");
      val_1 = 0; val_2 = 0;
    }
  }

  // Non-blocking accessor to cached encoder values (updated by reader thread)
  // Returns true if data copied (always true here unless disconnected)
  bool getLatestCachedEnc(std::array<int32_t,4> &out_enc)
  {
    std::lock_guard<std::mutex> lk(cached_enc_mtx_);
    out_enc = cached_enc_;
    return true;
  }

  // setMotorValues: non-blocking. Stores newest motor frame in prioritized slot (overrides previous motor commands)
  bool setMotorValues(long val_left, long val_right)
  {
    auto clamp = [](long v)->int8_t {
      if (v == 127) return static_cast<int8_t>(127);
      long c = std::max(-100L, std::min(100L, v));
      return static_cast<int8_t>(c);
    };

    int8_t left  = clamp(val_left);
    int8_t right = clamp(val_right);

    // Map: FL, FR, BL, BR
    int8_t s1 = left;
    int8_t s2 = left;
    int8_t s3 = right;
    int8_t s4 = right;

    std::vector<uint8_t> frame = build_set_motor(s1, s2, s3, s4);

    // overwrite latest motor frame atomically
    {
      std::lock_guard<std::mutex> lk(latest_motor_mtx_);
      latest_motor_frame_.assign(frame.begin(), frame.end());
      has_latest_motor_.store(true);
    }
    // wake writer thread immediately
    write_cv_.notify_one();
    return true;
  }

  // Enqueue non-motor frames (encoder requests, etc.)
  bool EnqueueWrite(const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> qlk(write_queue_mtx_);
    if (write_queue_.size() >= WRITE_QUEUE_MAX) {
      // drop the oldest frame to favour new frames
      write_queue_.pop_front();
      RCLCPP_WARN(rclcpp::get_logger("RosmasterController"),
                  "write queue full: dropping oldest frame to make room");
    }
    write_queue_.push_back(data);
    write_cv_.notify_one();
    return true;
  }

  // Emergency immediate estop: blocking write to ensure it reaches device now
  void EstopImmediate()
  {
    std::lock_guard<std::mutex> lk(write_mtx_);
    WriteToDriverUnlocked(build_set_motor(0,0,0,0));
  }

  // Soft estop: replace latest motor frame with zero (non-blocking)
  void Estop()
  {
    std::vector<uint8_t> frame = build_set_motor(0,0,0,0);
    {
      std::lock_guard<std::mutex> lk(latest_motor_mtx_);
      latest_motor_frame_.assign(frame.begin(), frame.end());
      has_latest_motor_.store(true);
    }
    write_cv_.notify_one();
  }

  bool connected() const {
    return serial_conn_.IsOpen();
  }

  // Non-blocking getEncoderValues: requests encoders and inspects read buffer filled by reader thread
  // (kept for compatibility; it performs a request/response using the read buffer)
  bool getEncoderValues(std::array<int32_t,4> &out_enc, unsigned int timeout_ms = 200)
  {
    if (!connected()) return false;

    // Clear stale bytes to avoid misalignment with old frames
    {
      std::lock_guard<std::mutex> lk(read_mtx_);
      if (!read_buffer_.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("RosmasterController"),
                     "Clearing %zu stale bytes from read buffer before request", read_buffer_.size());
        read_buffer_.clear();
      }
    }

    // send request frame (enqueue so non-blocking for caller context)
    std::vector<uint8_t> req = build_request_data(0x0D, 0x00);
    {
      std::lock_guard<std::mutex> lk(write_mtx_);
      if (!WriteToDriverUnlocked(req)) return false;
    }

    // tiny yield so reader thread can pick up bytes (short)
    std::this_thread::sleep_for(std::chrono::milliseconds(POLL_SLEEP_MS));

    const uint64_t deadline_ms = current_time_ms() + timeout_ms;

    while (current_time_ms() < deadline_ms) {
      std::vector<uint8_t> buf_copy;
      {
        std::lock_guard<std::mutex> lk(read_mtx_);
        if (read_buffer_.empty()) {
          // No bytes yet
        } else {
          buf_copy.assign(read_buffer_.begin(), read_buffer_.end());
        }
      }

      if (buf_copy.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_SLEEP_MS));
        continue;
      }

      // scan for complete frames in the buffer copy
      for (size_t start = 0; start < buf_copy.size(); ++start) {
        if (buf_copy[start] != HEAD) continue;

        // need at least 3 bytes (HEAD, CMD, EXT_LEN) to read ext_len
        if (start + 3 > buf_copy.size()) break;

        uint8_t ext_len = buf_copy[start + 2];

        // sanity check ext_len (protocol-dependent). require ext_len >= 2
        if (ext_len < 2) {
          // malformed ext_len, skip this HEAD and continue scanning
          continue;
        }

        // compute data length and total frame bytes we expect
        size_t data_len = static_cast<size_t>(ext_len - 2);
        size_t total_len = 4 + data_len; // HEAD + CMD + EXT + DATA + CRC? (match your protocol)

        // ensure we have the full frame bytes in the copy
        if (start + total_len > buf_copy.size()) {
          // not enough bytes yet, wait for more
          break;
        }

        // extract candidate frame
        std::vector<uint8_t> frame(buf_copy.begin() + start, buf_copy.begin() + start + total_len);
        auto maybe_enc = rosmaster::parse_encoder_frame(frame);
        if (maybe_enc) {
          // consume bytes up to start+total_len from the real buffer
          {
            std::lock_guard<std::mutex> lk(read_mtx_);
            size_t remove_cnt = start + total_len;
            for (size_t i=0; i<remove_cnt && !read_buffer_.empty(); ++i) read_buffer_.pop_front();
          }
          out_enc = *maybe_enc;
          return true;
        }
        // otherwise keep scanning
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(POLL_SLEEP_MS));
    }

    // timeout — helpful debug dump
    {
      std::lock_guard<std::mutex> lk(read_mtx_);
      size_t dump_len = std::min<size_t>(read_buffer_.size(), 128);
      std::ostringstream oss;
      oss << "Encoder read timeout: buffer size=" << read_buffer_.size() << " recent bytes (hex)=";
      auto it = read_buffer_.begin();
      for (size_t i = 0; i < dump_len && it != read_buffer_.end(); ++i, ++it) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(*it) << " ";
      }
      if (read_buffer_.size() > dump_len) oss << "...";
      RCLCPP_WARN(rclcpp::get_logger("RosmasterController"), "%s", oss.str().c_str());
    }

    return false; // timeout
  }

  // Blocking write API (keeps original semantics)
  bool WriteToDriver(const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lk(write_mtx_);
    return WriteToDriverUnlocked(data);
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_{200};

  // tuning constants
  static constexpr unsigned int READ_TIMEOUT_MS = 50;   // ms for ReadByte in reader thread
  static constexpr unsigned int POLL_SLEEP_MS   = 1;    // ms sleep in polling loops
  static constexpr size_t WRITE_QUEUE_MAX       = 128;  // bounded queue

  // separate mutexes for read/write
  std::mutex write_mtx_;
  std::mutex read_mtx_;

  // reader thread + buffer
  std::thread reader_thread_;
  std::atomic<bool> running_{false};
  std::deque<uint8_t> read_buffer_;

  // writer thread + normal queue
  std::thread writer_thread_;
  std::atomic<bool> writer_running_{false};
  std::deque<std::vector<uint8_t>> write_queue_;
  std::mutex write_queue_mtx_;
  std::condition_variable write_cv_;

  // latest motor frame slot (replaceable — only newest kept)
  std::vector<uint8_t> latest_motor_frame_;
  std::mutex latest_motor_mtx_;
  std::atomic<bool> has_latest_motor_{false};

  // cached latest encoder values (updated by reader thread)
  std::array<int32_t,4> cached_enc_{0,0,0,0};
  std::mutex cached_enc_mtx_;

  // reader loop: blocks on ReadByte and appends bytes to read_buffer_
  // when it recognizes an encoder frame it updates cached_enc_
  void readerLoop()
  {
    while (running_.load()) {
      try {
        unsigned char b = 0;
        // tuned shorter timeout to reduce read latency
        serial_conn_.ReadByte(b, static_cast<unsigned int>(READ_TIMEOUT_MS));
        {
          std::lock_guard<std::mutex> lk(read_mtx_);
          read_buffer_.push_back(static_cast<uint8_t>(b));
          if (read_buffer_.size() > 16384) read_buffer_.pop_front();
        }
      } catch (const LibSerial::ReadTimeout &) {
        // normal: no data this short window
        continue;
      } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("RosmasterController"),
                     "Serial reader thread error: %s", e.what());
        running_.store(false);
        break;
      }

      // Try to parse any complete encoder frames in the buffer and update cache.
      // We copy the buffer snapshot to avoid holding read_mtx_ while parsing.
      std::vector<uint8_t> buf_copy;
      {
        std::lock_guard<std::mutex> lk(read_mtx_);
        if (!read_buffer_.empty()) {
          buf_copy.assign(read_buffer_.begin(), read_buffer_.end());
        }
      }

      if (buf_copy.empty()) continue;

      for (size_t start = 0; start + 3 <= buf_copy.size(); ++start) {
        if (buf_copy[start] != HEAD) continue;
        if (start + 3 > buf_copy.size()) break;

        uint8_t ext_len = buf_copy[start + 2];
        if (ext_len < 2) continue;
        size_t data_len = static_cast<size_t>(ext_len - 2);
        size_t total_len = 4 + data_len;
        if (start + total_len > buf_copy.size()) break;

        // candidate frame
        std::vector<uint8_t> frame(buf_copy.begin() + start, buf_copy.begin() + start + total_len);
        auto maybe_enc = rosmaster::parse_encoder_frame(frame);
        if (maybe_enc) {
          // Update cached encoder values
          {
            std::lock_guard<std::mutex> lk(cached_enc_mtx_);
            cached_enc_ = *maybe_enc;
          }

          // consume bytes up to start+total_len from the real buffer
          {
            std::lock_guard<std::mutex> lk(read_mtx_);
            size_t remove_cnt = start + total_len;
            for (size_t i=0; i<remove_cnt && !read_buffer_.empty(); ++i) read_buffer_.pop_front();
          }

          // continue scanning from buffer start (we updated the real buffer)
          break; // break out of this for to re-copy buffer on next loop iteration
        }
      }
    }
  }

  // writer loop: prioritize latest motor frame, then normal queue
  void writerLoop()
  {
    while (writer_running_.load()) {
      std::vector<uint8_t> frame;

      // 1) Highest priority: latest motor frame
      if (has_latest_motor_.load()) {
        std::lock_guard<std::mutex> lk(latest_motor_mtx_);
        if (has_latest_motor_.load() && !latest_motor_frame_.empty()) {
          frame.assign(latest_motor_frame_.begin(), latest_motor_frame_.end());
          latest_motor_frame_.clear();
          has_latest_motor_.store(false);
        }
      }

      // 2) If no motor frame, pop from normal queue
      if (frame.empty()) {
        std::unique_lock<std::mutex> qlk(write_queue_mtx_);
        if (write_queue_.empty()) {
          // wait until we have something or timeout to re-check flags
          write_cv_.wait_for(qlk, std::chrono::milliseconds(100));
        }
        if (!write_queue_.empty()) {
          frame = std::move(write_queue_.front());
          write_queue_.pop_front();
        } else {
          continue;
        }
      }

      // perform actual write under serial lock
      {
        std::lock_guard<std::mutex> lk(write_mtx_);
        if (!WriteToDriverUnlocked(frame)) {
          RCLCPP_WARN(rclcpp::get_logger("RosmasterController"),
                      "writerLoop: failed to write frame (dropped)");
        }
      }
    }

    // flush remaining queued frames on shutdown
    {
      std::lock_guard<std::mutex> qlk(write_queue_mtx_);
      while (!write_queue_.empty()) {
        std::vector<uint8_t> frame = std::move(write_queue_.front());
        write_queue_.pop_front();
        std::lock_guard<std::mutex> lk(write_mtx_);
        WriteToDriverUnlocked(frame);
      }
    }

    // flush any leftover latest motor frame
    if (has_latest_motor_.load()) {
      std::lock_guard<std::mutex> lk(latest_motor_mtx_);
      if (!latest_motor_frame_.empty()) {
        std::lock_guard<std::mutex> lk2(write_mtx_);
        WriteToDriverUnlocked(latest_motor_frame_);
      }
      latest_motor_frame_.clear();
      has_latest_motor_.store(false);
    }
  }

  // write without acquiring external lock (caller must hold write_mtx_)
  bool WriteToDriverUnlocked(const std::vector<uint8_t> &data)
  {
    if (!serial_conn_.IsOpen()) return false;
    try {
      std::string s(reinterpret_cast<const char*>(data.data()), data.size());
      serial_conn_.Write(s);
      // avoid draining to reduce blocking latency
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("RosmasterController"),
                   "Serial write error: %s", e.what());
      return false;
    }
  }

  static uint64_t current_time_ms() {
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
  }

  // helper: convert common baudrates to LibSerial type
  inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
  {
    switch (baud_rate)
    {
      case 1200:   return LibSerial::BaudRate::BAUD_1200;
      case 1800:   return LibSerial::BaudRate::BAUD_1800;
      case 2400:   return LibSerial::BaudRate::BAUD_2400;
      case 4800:   return LibSerial::BaudRate::BAUD_4800;
      case 9600:   return LibSerial::BaudRate::BAUD_9600;
      case 19200:  return LibSerial::BaudRate::BAUD_19200;
      case 38400:  return LibSerial::BaudRate::BAUD_38400;
      case 57600:  return LibSerial::BaudRate::BAUD_57600;
      case 115200: return LibSerial::BaudRate::BAUD_115200;
      case 230400: return LibSerial::BaudRate::BAUD_230400;
      default:
        RCLCPP_WARN(rclcpp::get_logger("RosmasterController"),
                    "Unsupported baud rate %d; defaulting to 115200", baud_rate);
        return LibSerial::BaudRate::BAUD_115200;
    }
  }
};

#endif // ROSMASTER_CONTROLLER_HPP