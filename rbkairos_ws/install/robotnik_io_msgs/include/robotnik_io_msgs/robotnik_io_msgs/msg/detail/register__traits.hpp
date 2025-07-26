// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_io_msgs:msg/Register.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__TRAITS_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_io_msgs/msg/detail/register__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_io_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Register & msg,
  std::ostream & out)
{
  out << "{";
  // member: address
  {
    out << "address: ";
    rosidl_generator_traits::value_to_yaml(msg.address, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Register & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "address: ";
    rosidl_generator_traits::value_to_yaml(msg.address, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Register & msg, bool use_flow_style = false)
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

}  // namespace robotnik_io_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_io_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_io_msgs::msg::Register & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::msg::Register & msg)
{
  return robotnik_io_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::msg::Register>()
{
  return "robotnik_io_msgs::msg::Register";
}

template<>
inline const char * name<robotnik_io_msgs::msg::Register>()
{
  return "robotnik_io_msgs/msg/Register";
}

template<>
struct has_fixed_size<robotnik_io_msgs::msg::Register>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robotnik_io_msgs::msg::Register>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robotnik_io_msgs::msg::Register>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__TRAITS_HPP_
