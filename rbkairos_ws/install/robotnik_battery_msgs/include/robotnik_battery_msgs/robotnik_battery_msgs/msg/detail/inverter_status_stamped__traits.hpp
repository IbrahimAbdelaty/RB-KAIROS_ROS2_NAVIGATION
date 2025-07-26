// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_battery_msgs:msg/InverterStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__TRAITS_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_battery_msgs/msg/detail/inverter_status_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'status'
#include "robotnik_battery_msgs/msg/detail/inverter_status__traits.hpp"

namespace robotnik_battery_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const InverterStatusStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    to_flow_style_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InverterStatusStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status:\n";
    to_block_style_yaml(msg.status, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InverterStatusStamped & msg, bool use_flow_style = false)
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

}  // namespace robotnik_battery_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_battery_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_battery_msgs::msg::InverterStatusStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_battery_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_battery_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_battery_msgs::msg::InverterStatusStamped & msg)
{
  return robotnik_battery_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_battery_msgs::msg::InverterStatusStamped>()
{
  return "robotnik_battery_msgs::msg::InverterStatusStamped";
}

template<>
inline const char * name<robotnik_battery_msgs::msg::InverterStatusStamped>()
{
  return "robotnik_battery_msgs/msg/InverterStatusStamped";
}

template<>
struct has_fixed_size<robotnik_battery_msgs::msg::InverterStatusStamped>
  : std::integral_constant<bool, has_fixed_size<robotnik_battery_msgs::msg::InverterStatus>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<robotnik_battery_msgs::msg::InverterStatusStamped>
  : std::integral_constant<bool, has_bounded_size<robotnik_battery_msgs::msg::InverterStatus>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<robotnik_battery_msgs::msg::InverterStatusStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__TRAITS_HPP_
