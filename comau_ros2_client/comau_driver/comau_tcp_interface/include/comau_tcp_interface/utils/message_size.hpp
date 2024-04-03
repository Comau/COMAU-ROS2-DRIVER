/**
 * @file message_size.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#pragma once

#include <cstring>
#include <stdint.h>
#include <string>
#include <vector>

namespace comau_tcp_interface {
namespace utils {
/**
 * @brief The MessageSize class helping to meassure the size of message
 */
class MessageSize {
public:
  /**
   * @brief a general method for meassuring the size of arbitary datatypes
   *
   * @tparam T The type to serialize
   * @param val The input value
   * @return size_t Size in byte of the value
   */
  template <typename T> static size_t size(T val) {
    return sizeof(val);
  }

  template <typename T> static size_t size(std::vector<T> vec) {
    std::cout << "ALLERT! sizeof(vec) " << sizeof(vec) << ", vec.size() " << vec.size() << ", sizeof(T) " << sizeof(T)  << std::endl;
    // return sizeof(vec)*vec.size();
    return sizeof(T)*vec.size(); // VE_ATTENTION
  }

  static size_t size(std::string &str){
    return str.size();
  }

};

} // namespace utils

} // namespace comau_tcp_interface
