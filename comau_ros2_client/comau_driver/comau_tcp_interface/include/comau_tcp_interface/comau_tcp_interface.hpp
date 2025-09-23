/**
 * @file comau_tcp_interface.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that composes the low level of TCP message
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#ifndef COMAU_TCP_INTERFACE_H
#define COMAU_TCP_INTERFACE_H

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"
#include <comau_tcp_interface/utils/message_parser.hpp>

namespace comau_tcp_interface {

/**
 * @brief This is the structure that contains all the parameters that are used from
 * ComauTcpInterface class
 *
 */
struct ComauTcpInterfaceParameters {
  std::string server_ip_address; /**< Address of the Server that will be connected */
  std::string server_port;       /**< Port of the Server that will be connected */
  std::string log_tag;          /**< Tag that presented at output stream in brackets []*/
};

/**
 * @brief This is the main class that handles the TCP connection with the COMAU C5G contoller
 *
 */
class ComauTcpConnection {
public:
  /**
   * @brief Construct a new Comau Tcp Interface object
   *
   * @param params as defined in struct ComauTcpInterfaceParameters
   */
  ComauTcpConnection(const ComauTcpInterfaceParameters& params);
  /**
   * @brief Destroy the Comau Tcp Interface object
   *
   */
  ~ComauTcpConnection();

  /**
   * @brief The main function that connects with the PDL TCP server using the ComauTcpInterfaceParameters
   *
   */
  void connectToServer();

  /**
   * @brief Reads data from the socket
   *
   * @param buf [out] buf Buffer where the data shall be stored
   * @param buf_len [in] buf_len Number of bytes in the buffer
   * @param read  [out] read Number of bytes actually read
   */
  void read(std::vector<uint8_t>& buf, const size_t buf_len, size_t& read);

  /**
   * @brief Writes a buffer to the TCP socket
   *
   * @param buf The buffer to write from
   * @param buf_len The length to write
   * @param written A reference used to indicate how many bytes were written
   */
  void write(const std::vector<uint8_t>& buf, const size_t buf_len, size_t& written);

private:
  const ComauTcpInterfaceParameters& params_; /**< private variable for parameters*/

  boost::shared_ptr<boost::asio::io_service>
      io_service_ptr_; /**< The io_context class provides the core I/O functionality for users of the asynchronous I/O
                          objects */
  boost::shared_ptr<boost::asio::ip::tcp::socket>
      socket_ptr_; /**< The basic_stream_socket class template provides asynchronous and blocking stream-oriented socket
                      functionality */
};

} // namespace comau_tcp_interface

#endif // COMAU_TCP_INTERFACE_H
