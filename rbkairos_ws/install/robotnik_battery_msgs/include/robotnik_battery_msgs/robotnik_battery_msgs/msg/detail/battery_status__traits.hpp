// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_battery_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_battery_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_battery_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: level
  {
    out << "level: ";
    rosidl_generator_traits::value_to_yaml(msg.level, out);
    out << ", ";
  }

  // member: time_remaining
  {
    out << "time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.time_remaining, out);
    out << ", ";
  }

  // member: time_charging
  {
    out << "time_charging: ";
    rosidl_generator_traits::value_to_yaml(msg.time_charging, out);
    out << ", ";
  }

  // member: is_charging
  {
    out << "is_charging: ";
    rosidl_generator_traits::value_to_yaml(msg.is_charging, out);
    out << ", ";
  }

  // member: cell_voltages
  {
    if (msg.cell_voltages.size() == 0) {
      out << "cell_voltages: []";
    } else {
      out << "cell_voltages: [";
      size_t pending_items = msg.cell_voltages.size();
      for (auto item : msg.cell_voltages) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level: ";
    rosidl_generator_traits::value_to_yaml(msg.level, out);
    out << "\n";
  }

  // member: time_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.time_remaining, out);
    out << "\n";
  }

  // member: time_charging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_charging: ";
    rosidl_generator_traits::value_to_yaml(msg.time_charging, out);
    out << "\n";
  }

  // member: is_charging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_charging: ";
    rosidl_generator_traits::value_to_yaml(msg.is_charging, out);
    out << "\n";
  }

  // member: cell_voltages
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cell_voltages.size() == 0) {
      out << "cell_voltages: []\n";
    } else {
      out << "cell_voltages:\n";
      for (auto item : msg.cell_voltages) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BatteryStatus & msg, bool use_flow_style = false)
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
  const robotnik_battery_msgs::msg::BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_battery_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_battery_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_battery_msgs::msg::BatteryStatus & msg)
{
  return robotnik_battery_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_battery_msgs::msg::BatteryStatus>()
{
  return "robotnik_battery_msgs::msg::BatteryStatus";
}

template<>
inline const char * name<robotnik_battery_msgs::msg::BatteryStatus>()
{
  return "robotnik_battery_msgs/msg/BatteryStatus";
}

template<>
struct has_fixed_size<robotnik_battery_msgs::msg::BatteryStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_battery_msgs::msg::BatteryStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_battery_msgs::msg::BatteryStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
