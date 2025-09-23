/**
 * @file comau_data_type.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node creates the data type
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace comau_tcp_interface {

/**
 * @brief The RobotStatus class contains constants for status message field
 */
class RobotStatus {
public:
  inline static const char TERMINATE = 'T';
  inline static const char CANCELING = 'C';
  inline static const char READY = 'R';
  inline static const char MOVING = 'M';
  inline static const char RESUMING = 'I'; //??
  inline static const char PAUSED = 'P';
  inline static const char SUCCEEDED = 'S';
  inline static const char ERROR = 'E';
  inline static const char COLLISION = 'A';
};

/**
 * @brief The Server Error Value class contains constants for server (pdl) error value message field with client coding
 */
class ErrorValue {
public:
  inline static const uint32_t ERR_TCP_UNDEFINED     = 0x00001; /* Undefined      error */
  inline static const uint32_t ERR_TCP_CONN_STATE    = 0x00002; /* State  Server  error 15470 : Address already in use */
  inline static const uint32_t ERR_TCP_CONN_ROBOT    = 0x00004; /* Robot  Server  error 15470 : Address already in use */
  inline static const uint32_t ERR_TCP_CONN_ARM      = 0x00008; /* Motion Handler error 15470 : Address already in use */
  inline static const uint32_t ERR_TCP_READ          = 0x00010; /* State  Server  error 40033 : Error 15474 in write   */
  inline static const uint32_t ERR_TCP_WRITE_CMD     = 0x00020; /* Robot  Server  error 39990                          */
  inline static const uint32_t ERR_TCP_WRITE_MOTION  = 0x00040; /* Motion Handler error 39990                          */
  inline static const uint32_t ERR_TCP_DISCONN_STATE = 0x00080; /* State  Server  error 30767                          */
  inline static const uint32_t ERR_TCP_DISCONN_ROBOT = 0x00100; /* Robot  Server  error 30767                          */
  inline static const uint32_t ERR_TCP_DISCONN_ARM   = 0x00200; /* Motion Handler error 30767                          */
  inline static const uint32_t ERR_SAFETY_GATE       = 0x00400; /* Safety Gate / External Emergency Stop               */
  inline static const uint32_t ERR_WRONG_MOTION      = 0x00800; /*  Program Execution Errors (36864-37191)             */
  inline static const uint32_t ERR_ALARM             = 0x01000; /*               */
  inline static const uint32_t ERR_MOTION_DRIVEOFF   = 0x02000; /*               */
};

/**
 * @brief The MotionType class contains constants for motion_type message field
 */
class MotionType {
public:
  inline static const char JOINT_TRAJECTORY = 'T';
  inline static const char CARTESIAN_TRAJECTORY= 'C';
  inline static const char SENSOR_TRACKING = 'S';
  inline static const char JOINT_POSITION = 'J';
  inline static const char JOINT_MOVEFLY = 'F';
  inline static const char VELOCITY = 'V';
};
/**
 * @brief The RobotCommand class contains constants for robot set state message field
 */
class RobotCommand {
public:
  inline static const char INITIALIZE = 'I';
  inline static const char RESET = 'R';
  inline static const char DISCONNECT = 'D';
  inline static const char PAUSE = 'P';
  inline static const char START = 'S';
  inline static const char TERMINATE = 'T';
  inline static const char CANCEL = 'X';
  inline static const char CONFIGURATION = 'C';
  inline static const char IO = 'O';
  inline static const char LOCK = 'L';
  inline static const char UNLOCK = 'U';
};

namespace utils {

const bool False = false;
const bool True = false;

using vectorstr_t = std::vector<std::string>;
using vector6d_t  = std::array<double, 6>;
using vector6f_t  = std::array<float , 6>;

using vector6i_t  = std::array<int32_t, 6>;
using vector6b_t  = std::array<bool   , 6>;

using vector10f_t = std::array<float  , 10>;
using vector10i_t = std::array<int32_t, 10>;

typedef struct cart_trajectory_node
{
  vector6f_t pose;
  float      lin_vel;
  uint32_t   seg_ovr;
  int32_t    move_type;
}cart_traj_node;

typedef struct joint_trajectory_node
{
  vector10f_t pose;
  uint32_t   seg_ovr;
  int32_t    move_type;
}joint_traj_node;

using trajectoryd_t       = std::vector<vector6d_t>;
using trajectoryf_t       = std::vector<cart_traj_node>;  // Cartesian trajectory type
using joint_trajectoryf_t = std::vector<joint_traj_node>; // Joint trajectory type

/**
 * @brief Operator << overload for arrays
 *
 * @tparam T
 * @tparam N
 * @param out
 * @param item
 * @return std::ostream&
 */
template <class T, std::size_t N> std::ostream &operator<<(std::ostream &out, const std::array<T, N> &item) {
  out << "[";
  for (size_t i = 0; i < item.size(); ++i) {
    out << item[i];
    if (i != item.size() - 1) {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

/**
 * @brief Operator << overload for vectors
 *
 * @tparam T
 * @param os
 * @param v
 * @return std::ostream&
 */
template <typename T> std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "{";
  for (size_t i = 0; i < v.size(); ++i) {
    // os << v[i];
    if (i != v.size() - 1)
      os << ", ";
  }
  os << "}";
  return os;
}

/**
 * @brief Degree to rad conversion function for elements
 *
 * @tparam T
 * @param degree_elem
 * @return T
 */
template <class T> T toRad(const T &degree_elem) {
  T rad_elem;

    rad_elem = (double)(degree_elem * M_PI / 180);
  return rad_elem;
}

/**
 * @brief mm to m conversion function for elements
 *
 * @tparam T
 * @param mm_elem
 * @return T
 */
template <class T> T toMeter(const T &mm_elem) {
  T m_elem;

    m_elem = (double)(mm_elem * 0.001);
  return m_elem;
}

/**
 * @brief Degree to rad conversion function for arrays
 *
 * @tparam T
 * @tparam N
 * @param degree_vec
 * @return std::array<T, N>
 */
template <class T, std::size_t N> std::array<T, N> toRad(const std::array<T, N> &degree_vec) {
  std::array<T, N> rad_vec;
  for (size_t i = 0; i < degree_vec.size(); ++i) {
    rad_vec.at(i) = degree_vec.at(i) * M_PI / 180;
  }
  return rad_vec;
}

template <class T> std::vector<T> toRad(const std::vector<T> &degree_vec) {
  std::vector<T> rad_vec;
  for (size_t i = 0; i < degree_vec.size(); ++i) {
    rad_vec.at(i) = degree_vec.at(i) * M_PI / 180;
  }
  return rad_vec;
}


/**
 * @brief  Rad to degree conversion for arrays
 *
 * @tparam T
 * @tparam N
 * @param rad_vec
 * @return std::array<T, N>
 */
template <class T, std::size_t N> std::array<T, N> toDegree(const std::array<T, N> &rad_vec) {
  std::array<T, N> degree_vec;
  for (size_t i = 0; i < rad_vec.size(); ++i) {
    degree_vec.at(i) = rad_vec.at(i) * 180 / M_PI;
  }
  return degree_vec;
}

template <class T> std::vector<T> toDegree(const std::vector<T> &rad_vec) {
  std::vector<T> degree_vec;
  for (size_t i = 0; i < rad_vec.size(); ++i) {
    degree_vec.at(i) = rad_vec.at(i) * 180 / M_PI;
  }
  return degree_vec;
}



} // namespace utils
} // namespace comau_tcp_interface
