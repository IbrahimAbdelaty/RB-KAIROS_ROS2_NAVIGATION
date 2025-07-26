// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_controllers_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__TRAITS_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_controllers_msgs/msg/detail/controller_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace robotnik_controllers_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControllerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: state_number
  {
    out << "state_number: ";
    rosidl_generator_traits::value_to_yaml(msg.state_number, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControllerState & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: state_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_number: ";
    rosidl_generator_traits::value_to_yaml(msg.state_number, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControllerState & msg, bool use_flow_style = false)
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

}  // namespace robotnik_controllers_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_controllers_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_controllers_msgs::msg::ControllerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::msg::ControllerState & msg)
{
  return robotnik_controllers_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::msg::ControllerState>()
{
  return "robotnik_controllers_msgs::msg::ControllerState";
}

template<>
inline const char * name<robotnik_controllers_msgs::msg::ControllerState>()
{
  return "robotnik_controllers_msgs/msg/ControllerState";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::msg::ControllerState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::msg::ControllerState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_controllers_msgs::msg::ControllerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__TRAITS_HPP_
