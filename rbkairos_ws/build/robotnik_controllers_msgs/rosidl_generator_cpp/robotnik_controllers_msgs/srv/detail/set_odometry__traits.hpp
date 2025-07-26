// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_controllers_msgs:srv/SetOdometry.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__TRAITS_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_controllers_msgs/srv/detail/set_odometry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_controllers_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetOdometry_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetOdometry_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetOdometry_Request & msg, bool use_flow_style = false)
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

}  // namespace robotnik_controllers_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_controllers_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_controllers_msgs::srv::SetOdometry_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::srv::SetOdometry_Request & msg)
{
  return robotnik_controllers_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::SetOdometry_Request>()
{
  return "robotnik_controllers_msgs::srv::SetOdometry_Request";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::SetOdometry_Request>()
{
  return "robotnik_controllers_msgs/srv/SetOdometry_Request";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::SetOdometry_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::SetOdometry_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robotnik_controllers_msgs::srv::SetOdometry_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__traits.hpp"

namespace robotnik_controllers_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetOdometry_Response & msg,
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
  const SetOdometry_Response & msg,
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

inline std::string to_yaml(const SetOdometry_Response & msg, bool use_flow_style = false)
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

}  // namespace robotnik_controllers_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_controllers_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_controllers_msgs::srv::SetOdometry_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::srv::SetOdometry_Response & msg)
{
  return robotnik_controllers_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::SetOdometry_Response>()
{
  return "robotnik_controllers_msgs::srv::SetOdometry_Response";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::SetOdometry_Response>()
{
  return "robotnik_controllers_msgs/srv/SetOdometry_Response";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::SetOdometry_Response>
  : std::integral_constant<bool, has_fixed_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::SetOdometry_Response>
  : std::integral_constant<bool, has_bounded_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct is_message<robotnik_controllers_msgs::srv::SetOdometry_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::SetOdometry>()
{
  return "robotnik_controllers_msgs::srv::SetOdometry";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::SetOdometry>()
{
  return "robotnik_controllers_msgs/srv/SetOdometry";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::SetOdometry>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_controllers_msgs::srv::SetOdometry_Request>::value &&
    has_fixed_size<robotnik_controllers_msgs::srv::SetOdometry_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::SetOdometry>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_controllers_msgs::srv::SetOdometry_Request>::value &&
    has_bounded_size<robotnik_controllers_msgs::srv::SetOdometry_Response>::value
  >
{
};

template<>
struct is_service<robotnik_controllers_msgs::srv::SetOdometry>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_controllers_msgs::srv::SetOdometry_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_controllers_msgs::srv::SetOdometry_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__TRAITS_HPP_
