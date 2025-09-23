/**
 * @file message_serializer.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node serializes the TCP message
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
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
 * @brief A helper class to serialize messages
 *
 */
class MessageSerializer {
public:
  /**
   * @brief A generalized serialization method for arbitrary datatypes.
   *
   * @tparam T The type to serialize
   * @param buffer The buffer to write the serialization into
   * @param val The value to serialize
   * @return size_t Size in byte of the serialization
   */
  template <typename T> static size_t serialize(uint8_t* buffer, T val) {
    size_t size = sizeof(T);
    std::memcpy(buffer, &val, size);
    return size;
  }
  /**
   * @brief A generalized serialization method for vector of arbitrary datatypes.
   *
   * @tparam T The type to serialize
   * @param buffer The buffer to write the serialization into
   * @param val The value to serialize
   * @return size_t Size in byte of the serialization
   */
  template <typename T> static size_t serialize(uint8_t* buffer, std::vector<T> vec) {
    size_t size = 0;

    for (auto &item : vec) {
      size += serialize(buffer + size,item);
//      buffer += sizeof(item);
    }
    return size;
  }
  /**
   * @brief A serialization method for double values.
   *
   * @param buffer The buffer to write the serialization into.
   * @param val The value to serialize.
   * @return size_t size_t Size in byte of the serialization
   */
  static size_t serialize(uint8_t* buffer, double val) {
    size_t size = sizeof(double);
    uint64_t inner;
    std::memcpy(&inner, &val, size);
    std::memcpy(buffer, &inner, size);
    return size;
  }
  /**
   * @brief A serialization method for float values.
   *
   * @param buffer The buffer to write the serialization into.
   * @param val The value to serialize.
   * @return size_t size_t Size in byte of the serialization
   */
  static size_t serialize(uint8_t* buffer, float val) {
    size_t size = sizeof(float);
    uint32_t inner;
    std::memcpy(&inner, &val, size);
    std::memcpy(buffer, &inner, size);
    return size;
  }
  /**
   * @brief A serialization method for string values.
   *
   * @param buffer The buffer to write the serialization into.
   * @param val The value to serialize.
   * @return size_t size_t Size in byte of the serialization
   */
  static size_t serialize(uint8_t* buffer, std::string val) {
    const uint8_t* c_val = reinterpret_cast<const uint8_t*>(val.c_str());

    for (size_t i = 0; i < val.size(); i++) {
      buffer[i] = c_val[i];
    }
    return val.size();
  }


private:
  /* data */
};

} // namespace utils
} // namespace comau_tcp_interface
