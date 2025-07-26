// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_battery_msgs:msg/InverterStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__TRAITS_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_battery_msgs/msg/detail/inverter_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_battery_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const InverterStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: ac_voltage
  {
    out << "ac_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.ac_voltage, out);
    out << ", ";
  }

  // member: dc_voltage
  {
    out << "dc_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.dc_voltage, out);
    out << ", ";
  }

  // member: load
  {
    out << "load: ";
    rosidl_generator_traits::value_to_yaml(msg.load, out);
    out << ", ";
  }

  // member: percentage
  {
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: on
  {
    out << "on: ";
    rosidl_generator_traits::value_to_yaml(msg.on, out);
    out << ", ";
  }

  // member: serial_number
  {
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InverterStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ac_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ac_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.ac_voltage, out);
    out << "\n";
  }

  // member: dc_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dc_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.dc_voltage, out);
    out << "\n";
  }

  // member: load
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "load: ";
    rosidl_generator_traits::value_to_yaml(msg.load, out);
    out << "\n";
  }

  // member: percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.percentage, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: on
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "on: ";
    rosidl_generator_traits::value_to_yaml(msg.on, out);
    out << "\n";
  }

  // member: serial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InverterStatus & msg, bool use_flow_style = false)
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
  const robotnik_battery_msgs::msg::InverterStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_battery_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_battery_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_battery_msgs::msg::InverterStatus & msg)
{
  return robotnik_battery_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_battery_msgs::msg::InverterStatus>()
{
  return "robotnik_battery_msgs::msg::InverterStatus";
}

template<>
inline const char * name<robotnik_battery_msgs::msg::InverterStatus>()
{
  return "robotnik_battery_msgs/msg/InverterStatus";
}

template<>
struct has_fixed_size<robotnik_battery_msgs::msg::InverterStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_battery_msgs::msg::InverterStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_battery_msgs::msg::InverterStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__TRAITS_HPP_
