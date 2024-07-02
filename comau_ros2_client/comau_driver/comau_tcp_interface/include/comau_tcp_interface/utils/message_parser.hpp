/**
 * @file message_parser.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node parses the TCP message
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#pragma once

#include <assert.h>
#include <cmath>
#include <cstring>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"

#include "comau_tcp_interface/utils/custom_data_type.hpp"

namespace comau_tcp_interface {
namespace utils {

/**
 * @brief The MessageParser class handles the binary message parsing
 *
 */
class MessageParser {
public:
  /**
   * @brief Construct a new Message Parser object
   *
   * @param buffer
   * @param buffer_len
   */
  MessageParser(uint8_t* buffer, size_t buffer_len)
      : buffer_curr_pos_(buffer), buffer_end_(buffer + buffer_len), parent_(*this) {
    assert(buffer_curr_pos_ <= buffer_end_);
  }
  /**
   * @brief Destroy the Message Parser object
   *
   */
  ~MessageParser() {
    parent_.buffer_curr_pos_ = buffer_curr_pos_;
  }

  /**
   * @brief Parses the next bytes as given type without moving the buffer pointer.
   *
   * @tparam T Type to parse as
   * @return Value of the next bytes as type T
   */
  template <typename T> T peek() {
    if (buffer_curr_pos_ + sizeof(T) > buffer_end_)
    {
      //RCLCPP_ERROR_STREAM("SEVERE ERROR: " << &buffer_curr_pos_ << " + " << sizeof(T) << " > " << &buffer_end_);
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("SEVERE ERROR: "), buffer_curr_pos_ <<  " + " << sizeof(T) << " > " << buffer_end_);
      //throw new std::string("Could not parse received package.");
    }
    T value;
    std::memcpy(&value, buffer_curr_pos_, sizeof(T));
    return value;
  }
  /**
   * @brief Parses the next bytes as given type.
   *
   * @param value Reference to write the parsed value to
   */
  template <typename T> void parse(T& value) {
    value = peek<T>();
    buffer_curr_pos_ += sizeof(T);
  }
  /**
   * @brief Parses the next bytes as double
   *
   * @param value Reference to write the parsed value to
   */
  void parse(double& value) {
    uint64_t inner;
    parse<uint64_t>(inner);
    std::memcpy(&value, &inner, sizeof(double));
  }
  /**
   * @brief Parses the next bytes as float
   *
   * @param value Reference to write the parsed value to
   */
  void parse(float& value) {
    uint32_t inner;
    parse<uint32_t>(inner);
    std::memcpy(&value, &inner, sizeof(float));
  }
  /**
   * @brief Parses the next byte as a bool.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(bool& value) {
    uint8_t inner;
    parse<uint8_t>(inner);
    value = (inner != 0);
  }
  /**
   * @brief Parses the next bytes as a vector of 6 doubles.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector6d_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the next bytes as a vector of 6 floats.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector6f_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the next bytes as a vector of 6 ints.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector6i_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the next bytes as a vector of 10 ints.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector10i_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the next bytes as a vector of 10 float.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector10f_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the next bytes as a vector of 6 bools.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(vector6b_t& value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the dynamic size trajectory.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(trajectoryf_t &value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses the dynamic size trajectory.
   *
   * @param value Reference to write the parsed value to
   */
  void parse(trajectoryd_t &value) {
    for (size_t i = 0; i < value.size(); ++i) {
      parse(value[i]);
    }
  }
  /**
   * @brief Parses a given number of bytes as a string.
   *
   * @param value Reference to write the parsed value to
   * @param length  Number of bytes to parse
   */
  void parse(std::string& value, size_t length) {
    value.assign(reinterpret_cast<char*>(buffer_curr_pos_), length);
    buffer_curr_pos_ += length;
  }
  /**
   * @brief Parses the remaining bytes as a string.
   *
   * @param value Reference to write the parsed value to
   */
  void parseRemainder(std::string& value) {
    parse(value, size_t(buffer_end_ - buffer_curr_pos_));
  }

private:
  uint8_t *buffer_curr_pos_, *buffer_end_;
  MessageParser& parent_;
};

} // namespace utils
} // namespace comau_tcp_interface
