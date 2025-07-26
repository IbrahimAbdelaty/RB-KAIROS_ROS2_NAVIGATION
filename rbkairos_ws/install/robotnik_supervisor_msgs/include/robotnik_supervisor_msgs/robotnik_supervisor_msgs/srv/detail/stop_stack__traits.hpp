// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_supervisor_msgs:srv/StopStack.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__STOP_STACK__TRAITS_HPP_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__STOP_STACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_supervisor_msgs/srv/detail/stop_stack__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_supervisor_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const StopStack_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopStack_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopStack_Request & msg, bool use_flow_style = false)
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

}  // namespace robotnik_supervisor_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_supervisor_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_supervisor_msgs::srv::StopStack_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_supervisor_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_supervisor_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_supervisor_msgs::srv::StopStack_Request & msg)
{
  return robotnik_supervisor_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_supervisor_msgs::srv::StopStack_Request>()
{
  return "robotnik_supervisor_msgs::srv::StopStack_Request";
}

template<>
inline const char * name<robotnik_supervisor_msgs::srv::StopStack_Request>()
{
  return "robotnik_supervisor_msgs/srv/StopStack_Request";
}

template<>
struct has_fixed_size<robotnik_supervisor_msgs::srv::StopStack_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_supervisor_msgs::srv::StopStack_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_supervisor_msgs::srv::StopStack_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__traits.hpp"

namespace robotnik_supervisor_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const StopStack_Response & msg,
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
  const StopStack_Response & msg,
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

inline std::string to_yaml(const StopStack_Response & msg, bool use_flow_style = false)
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

}  // namespace robotnik_supervisor_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_supervisor_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_supervisor_msgs::srv::StopStack_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_supervisor_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_supervisor_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_supervisor_msgs::srv::StopStack_Response & msg)
{
  return robotnik_supervisor_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_supervisor_msgs::srv::StopStack_Response>()
{
  return "robotnik_supervisor_msgs::srv::StopStack_Response";
}

template<>
inline const char * name<robotnik_supervisor_msgs::srv::StopStack_Response>()
{
  return "robotnik_supervisor_msgs/srv/StopStack_Response";
}

template<>
struct has_fixed_size<robotnik_supervisor_msgs::srv::StopStack_Response>
  : std::integral_constant<bool, has_fixed_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct has_bounded_size<robotnik_supervisor_msgs::srv::StopStack_Response>
  : std::integral_constant<bool, has_bounded_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct is_message<robotnik_supervisor_msgs::srv::StopStack_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_supervisor_msgs::srv::StopStack>()
{
  return "robotnik_supervisor_msgs::srv::StopStack";
}

template<>
inline const char * name<robotnik_supervisor_msgs::srv::StopStack>()
{
  return "robotnik_supervisor_msgs/srv/StopStack";
}

template<>
struct has_fixed_size<robotnik_supervisor_msgs::srv::StopStack>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_supervisor_msgs::srv::StopStack_Request>::value &&
    has_fixed_size<robotnik_supervisor_msgs::srv::StopStack_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_supervisor_msgs::srv::StopStack>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_supervisor_msgs::srv::StopStack_Request>::value &&
    has_bounded_size<robotnik_supervisor_msgs::srv::StopStack_Response>::value
  >
{
};

template<>
struct is_service<robotnik_supervisor_msgs::srv::StopStack>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_supervisor_msgs::srv::StopStack_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_supervisor_msgs::srv::StopStack_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__STOP_STACK__TRAITS_HPP_
