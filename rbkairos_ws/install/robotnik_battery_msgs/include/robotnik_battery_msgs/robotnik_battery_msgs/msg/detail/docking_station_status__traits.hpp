// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_battery_msgs:msg/DockingStationStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__TRAITS_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_battery_msgs/msg/detail/docking_station_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_battery_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DockingStationStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: operation_mode
  {
    out << "operation_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_mode, out);
    out << ", ";
  }

  // member: contact_relay_status
  {
    out << "contact_relay_status: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_relay_status, out);
    out << ", ";
  }

  // member: charger_relay_status
  {
    out << "charger_relay_status: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_relay_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DockingStationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: operation_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_mode, out);
    out << "\n";
  }

  // member: contact_relay_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_relay_status: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_relay_status, out);
    out << "\n";
  }

  // member: charger_relay_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charger_relay_status: ";
    rosidl_generator_traits::value_to_yaml(msg.charger_relay_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DockingStationStatus & msg, bool use_flow_style = false)
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
  const robotnik_battery_msgs::msg::DockingStationStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_battery_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_battery_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_battery_msgs::msg::DockingStationStatus & msg)
{
  return robotnik_battery_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_battery_msgs::msg::DockingStationStatus>()
{
  return "robotnik_battery_msgs::msg::DockingStationStatus";
}

template<>
inline const char * name<robotnik_battery_msgs::msg::DockingStationStatus>()
{
  return "robotnik_battery_msgs/msg/DockingStationStatus";
}

template<>
struct has_fixed_size<robotnik_battery_msgs::msg::DockingStationStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_battery_msgs::msg::DockingStationStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_battery_msgs::msg::DockingStationStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__TRAITS_HPP_
