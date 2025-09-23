/**
 * @file message_package.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node packages the TCP message
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#pragma once

#include <boost/variant.hpp>
#include <variant>

#include "comau_tcp_interface/utils/custom_data_type.hpp"
#include "comau_tcp_interface/utils/message_parser.hpp"
#include "comau_tcp_interface/utils/message_serializer.hpp"
#include "comau_tcp_interface/utils/message_size.hpp"

namespace comau_tcp_interface {
namespace utils {

/**
 * @brief The MessagePackage class descibes the message between PDL server and C++ client
 *
 */
class MessagePackage {
public:
  using _type_variants = boost::variant<bool, char, uint8_t, uint32_t, int32_t, 
                                        float, double, vector6f_t, vector6d_t,
                                        vector6i_t, vector6b_t, vector10i_t, vector10f_t, 
                                        std::string, trajectoryf_t, trajectoryd_t, uint32_t, joint_trajectoryf_t>;
  /**
   * @brief Construct a new Message Package object based on the given description
   *
   * @param description
   */
  MessagePackage(const vectorstr_t &description) : descr_(description) {
    id_ = next_id_++;
  }
  /**
   * @brief Copy constructor
   */
  MessagePackage(const MessagePackage &msg) {
    id_ = next_id_++;
    descr_ = msg.descr_;
    data_ = msg.data_;
  }
  /**
   * @brief Destroy the Message Package object
   *
   */
  virtual ~MessagePackage() = default;
  /**
   * @brief Put in msg_ zero vallues of the data that are defined in description
   *
   */
  void setZero();
  /**
   * @brief getCapacity Returns the maximum size that can be stored in this package based on its description
   * @return
   */
  size_t getCapacity() const;
  /**
   * @brief getSize Returns the size of message package
   * @return
   */
  size_t getSize() const;
  /**
   * @brief Sets the attributes of the message by parsing a serialized representation of the
   * package.
   *
   * @param mp A parser containing a serialized version of the package
   * @return True, if the package was parsed successfully, false otherwise
   */
  bool parseWith(MessageParser &mp);
  /**
   * @brief Produces a human readable representation of the message.
   *
   * @return std::string
   */
  virtual std::string toString();
  /**
   * @brief Serializes the package.
   *
   * @param buffer Buffer to fill with the serialization
   * @return size_t The total size of the serialized package
   */
  size_t serializePackage(uint8_t *buffer);
  /**
   * @brief Get field from the Message Package
   *  The message package contains a lot of different data fields, depending on the description.
   *
   * @param name The string identifier for the data field as used in the documentation.
   * @param value Target variable. Make sure, it's the correct type.
   * @return True on success, false if the field cannot be found inside the package.
   * @exception std::e if the type under given \p name does not match the template type T.
   */
  template <typename T> bool getData(const std::string &name, T &value) {
    if (data_.find(name) != data_.end()) {
      try {
        value = boost::strict_get<T>(data_[name]);
      } catch (const boost::bad_get &e) {
        std::cerr << "MessagePackage::getData throw an excpetion at getData: " << e.what() << std::endl;
        return false;
      }
    } else {
      return false;
    }
    return true;
  }
  /**
   * @brief Set the a data field in the Message package
   *
   * @tparam T
   * @param name
   * @param value
   * @return true
   * @return false
   */
  template <typename T> bool setData(const std::string &name, T &value) {
    if (data_.find(name) != data_.end()) {
      data_[name] = value;
    } else {
      if (std::find(descr_.begin(), descr_.end(), name) != descr_.end()) {
        //        _type_variants entry = message_type_list_[name];
        //        data_[name] = entry;
        data_[name] = value;
      } else {
        std::cerr << "Message type name '" << name << "' not found in message description \n";
        return false;
      }
    }
    return true;
  }

private:
  static std::unordered_map<std::string, _type_variants> message_type_list_; /**< static map for containing the relation between message type and variable type >*/

  uint32_t id_; /**< private variable for the unique id of the message */

  std::vector<std::string> descr_; /**< private variable defining the structure of message*/

  std::unordered_map<std::string, _type_variants> data_; /**< the actual message data */

  static uint32_t next_id_; /**< static variable for giving the next id in each message instance*/
  /**
   * @brief The ParseVisitor struct
   */
  struct ParseVisitor : public boost::static_visitor<> {
    template <typename T> void operator()(T &d, MessageParser &mp) const {
      mp.parse(d);
    }
  };
  /**
   * @brief The StringVisitor struct
   */
  struct StringVisitor : public boost::static_visitor<std::string> {
    template <typename T> std::string operator()(T &d) const {
      std::stringstream ss;
      ss << d;
      return ss.str();
    }
  };
  /**
   * @brief The SizeVisitor struct
   */
  struct SizeVisitor : public boost::static_visitor<size_t> {

    template <typename T> size_t operator()(T &d) const {
      return MessageSize::size(d);
    }
  };
  /**
   * @brief The SerializeVisitor struct
   */
  struct SerializeVisitor : public boost::static_visitor<size_t> {
    template <typename T> size_t operator()(T &d, uint8_t *buffer) const {
      return MessageSerializer::serialize(buffer, d);
    }
  };
};

} // namespace utils
} // namespace comau_tcp_interface
