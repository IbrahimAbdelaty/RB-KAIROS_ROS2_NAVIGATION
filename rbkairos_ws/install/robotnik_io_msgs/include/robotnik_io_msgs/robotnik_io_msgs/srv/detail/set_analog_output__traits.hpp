// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_io_msgs:srv/SetAnalogOutput.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__TRAITS_HPP_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_io_msgs/srv/detail/set_analog_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'output'
#include "robotnik_io_msgs/msg/detail/analog_io__traits.hpp"

namespace robotnik_io_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetAnalogOutput_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: output
  {
    out << "output: ";
    to_flow_style_yaml(msg.output, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetAnalogOutput_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output:\n";
    to_block_style_yaml(msg.output, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetAnalogOutput_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robotnik_io_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_io_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_io_msgs::srv::SetAnalogOutput_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::srv::SetAnalogOutput_Request & msg)
{
  return robotnik_io_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::srv::SetAnalogOutput_Request>()
{
  return "robotnik_io_msgs::srv::SetAnalogOutput_Request";
}

template<>
inline const char * name<robotnik_io_msgs::srv::SetAnalogOutput_Request>()
{
  return "robotnik_io_msgs/srv/SetAnalogOutput_Request";
}

template<>
struct has_fixed_size<robotnik_io_msgs::srv::SetAnalogOutput_Request>
  : std::integral_constant<bool, has_fixed_size<robotnik_io_msgs::msg::AnalogIO>::value> {};

template<>
struct has_bounded_size<robotnik_io_msgs::srv::SetAnalogOutput_Request>
  : std::integral_constant<bool, has_bounded_size<robotnik_io_msgs::msg::AnalogIO>::value> {};

template<>
struct is_message<robotnik_io_msgs::srv::SetAnalogOutput_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__traits.hpp"

namespace robotnik_io_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetAnalogOutput_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    to_flow_style_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetAnalogOutput_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response:\n";
    to_block_style_yaml(msg.response, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetAnalogOutput_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robotnik_io_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_io_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_io_msgs::srv::SetAnalogOutput_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::srv::SetAnalogOutput_Response & msg)
{
  return robotnik_io_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::srv::SetAnalogOutput_Response>()
{
  return "robotnik_io_msgs::srv::SetAnalogOutput_Response";
}

template<>
inline const char * name<robotnik_io_msgs::srv::SetAnalogOutput_Response>()
{
  return "robotnik_io_msgs/srv/SetAnalogOutput_Response";
}

template<>
struct has_fixed_size<robotnik_io_msgs::srv::SetAnalogOutput_Response>
  : std::integral_constant<bool, has_fixed_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct has_bounded_size<robotnik_io_msgs::srv::SetAnalogOutput_Response>
  : std::integral_constant<bool, has_bounded_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct is_message<robotnik_io_msgs::srv::SetAnalogOutput_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_io_msgs::srv::SetAnalogOutput>()
{
  return "robotnik_io_msgs::srv::SetAnalogOutput";
}

template<>
inline const char * name<robotnik_io_msgs::srv::SetAnalogOutput>()
{
  return "robotnik_io_msgs/srv/SetAnalogOutput";
}

template<>
struct has_fixed_size<robotnik_io_msgs::srv::SetAnalogOutput>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_io_msgs::srv::SetAnalogOutput_Request>::value &&
    has_fixed_size<robotnik_io_msgs::srv::SetAnalogOutput_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_io_msgs::srv::SetAnalogOutput>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_io_msgs::srv::SetAnalogOutput_Request>::value &&
    has_bounded_size<robotnik_io_msgs::srv::SetAnalogOutput_Response>::value
  >
{
};

template<>
struct is_service<robotnik_io_msgs::srv::SetAnalogOutput>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_io_msgs::srv::SetAnalogOutput_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_io_msgs::srv::SetAnalogOutput_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT__TRAITS_HPP_
