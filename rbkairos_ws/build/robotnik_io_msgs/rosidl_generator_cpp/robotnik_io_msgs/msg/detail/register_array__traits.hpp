// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__TRAITS_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_io_msgs/msg/detail/register_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'registers'
#include "robotnik_io_msgs/msg/detail/register__traits.hpp"

namespace robotnik_io_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RegisterArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: registers
  {
    if (msg.registers.size() == 0) {
      out << "registers: []";
    } else {
      out << "registers: [";
      size_t pending_items = msg.registers.size();
      for (auto item : msg.registers) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RegisterArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: registers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.registers.size() == 0) {
      out << "registers: []\n";
    } else {
      out << "registers:\n";
      for (auto item : msg.registers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RegisterArray & msg, bool use_flow_style = false)
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
  const robotnik_io_msgs::msg::RegisterArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::msg::RegisterArray & msg)
{
  return robotnik_io_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::msg::RegisterArray>()
{
  return "robotnik_io_msgs::msg::RegisterArray";
}

template<>
inline const char * name<robotnik_io_msgs::msg::RegisterArray>()
{
  return "robotnik_io_msgs/msg/RegisterArray";
}

template<>
struct has_fixed_size<robotnik_io_msgs::msg::RegisterArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_io_msgs::msg::RegisterArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_io_msgs::msg::RegisterArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__TRAITS_HPP_
