// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_common_msgs:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__TRAITS_HPP_
#define ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_common_msgs/msg/detail/response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_common_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robotnik_common_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_common_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_common_msgs::msg::Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_common_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_common_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_common_msgs::msg::Response & msg)
{
  return robotnik_common_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_common_msgs::msg::Response>()
{
  return "robotnik_common_msgs::msg::Response";
}

template<>
inline const char * name<robotnik_common_msgs::msg::Response>()
{
  return "robotnik_common_msgs/msg/Response";
}

template<>
struct has_fixed_size<robotnik_common_msgs::msg::Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_common_msgs::msg::Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_common_msgs::msg::Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__TRAITS_HPP_
