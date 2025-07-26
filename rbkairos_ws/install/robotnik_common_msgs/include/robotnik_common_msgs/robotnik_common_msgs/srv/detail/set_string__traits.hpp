// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_common_msgs:srv/SetString.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_STRING__TRAITS_HPP_
#define ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_STRING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_common_msgs/srv/detail/set_string__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_common_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetString_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetString_Request & msg, bool use_flow_style = false)
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

}  // namespace robotnik_common_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_common_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_common_msgs::srv::SetString_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_common_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_common_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_common_msgs::srv::SetString_Request & msg)
{
  return robotnik_common_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_common_msgs::srv::SetString_Request>()
{
  return "robotnik_common_msgs::srv::SetString_Request";
}

template<>
inline const char * name<robotnik_common_msgs::srv::SetString_Request>()
{
  return "robotnik_common_msgs/srv/SetString_Request";
}

template<>
struct has_fixed_size<robotnik_common_msgs::srv::SetString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_common_msgs::srv::SetString_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_common_msgs::srv::SetString_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__traits.hpp"

namespace robotnik_common_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetString_Response & msg,
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
  const SetString_Response & msg,
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

inline std::string to_yaml(const SetString_Response & msg, bool use_flow_style = false)
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

}  // namespace robotnik_common_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_common_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_common_msgs::srv::SetString_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_common_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_common_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_common_msgs::srv::SetString_Response & msg)
{
  return robotnik_common_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_common_msgs::srv::SetString_Response>()
{
  return "robotnik_common_msgs::srv::SetString_Response";
}

template<>
inline const char * name<robotnik_common_msgs::srv::SetString_Response>()
{
  return "robotnik_common_msgs/srv/SetString_Response";
}

template<>
struct has_fixed_size<robotnik_common_msgs::srv::SetString_Response>
  : std::integral_constant<bool, has_fixed_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct has_bounded_size<robotnik_common_msgs::srv::SetString_Response>
  : std::integral_constant<bool, has_bounded_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct is_message<robotnik_common_msgs::srv::SetString_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_common_msgs::srv::SetString>()
{
  return "robotnik_common_msgs::srv::SetString";
}

template<>
inline const char * name<robotnik_common_msgs::srv::SetString>()
{
  return "robotnik_common_msgs/srv/SetString";
}

template<>
struct has_fixed_size<robotnik_common_msgs::srv::SetString>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_common_msgs::srv::SetString_Request>::value &&
    has_fixed_size<robotnik_common_msgs::srv::SetString_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_common_msgs::srv::SetString>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_common_msgs::srv::SetString_Request>::value &&
    has_bounded_size<robotnik_common_msgs::srv::SetString_Response>::value
  >
{
};

template<>
struct is_service<robotnik_common_msgs::srv::SetString>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_common_msgs::srv::SetString_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_common_msgs::srv::SetString_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_STRING__TRAITS_HPP_
