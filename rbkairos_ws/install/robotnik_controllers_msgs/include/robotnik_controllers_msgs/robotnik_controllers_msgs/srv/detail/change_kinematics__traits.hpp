// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_controllers_msgs:srv/ChangeKinematics.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__CHANGE_KINEMATICS__TRAITS_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__CHANGE_KINEMATICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_controllers_msgs/srv/detail/change_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_controllers_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ChangeKinematics_Request & msg,
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
  const ChangeKinematics_Request & msg,
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

inline std::string to_yaml(const ChangeKinematics_Request & msg, bool use_flow_style = false)
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
  const robotnik_controllers_msgs::srv::ChangeKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::srv::ChangeKinematics_Request & msg)
{
  return robotnik_controllers_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::ChangeKinematics_Request>()
{
  return "robotnik_controllers_msgs::srv::ChangeKinematics_Request";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::ChangeKinematics_Request>()
{
  return "robotnik_controllers_msgs/srv/ChangeKinematics_Request";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::ChangeKinematics_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::ChangeKinematics_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_controllers_msgs::srv::ChangeKinematics_Request>
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
  const ChangeKinematics_Response & msg,
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
  const ChangeKinematics_Response & msg,
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

inline std::string to_yaml(const ChangeKinematics_Response & msg, bool use_flow_style = false)
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
  const robotnik_controllers_msgs::srv::ChangeKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::srv::ChangeKinematics_Response & msg)
{
  return robotnik_controllers_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::ChangeKinematics_Response>()
{
  return "robotnik_controllers_msgs::srv::ChangeKinematics_Response";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::ChangeKinematics_Response>()
{
  return "robotnik_controllers_msgs/srv/ChangeKinematics_Response";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::ChangeKinematics_Response>
  : std::integral_constant<bool, has_fixed_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::ChangeKinematics_Response>
  : std::integral_constant<bool, has_bounded_size<robotnik_common_msgs::msg::Response>::value> {};

template<>
struct is_message<robotnik_controllers_msgs::srv::ChangeKinematics_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::ChangeKinematics>()
{
  return "robotnik_controllers_msgs::srv::ChangeKinematics";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::ChangeKinematics>()
{
  return "robotnik_controllers_msgs/srv/ChangeKinematics";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::ChangeKinematics>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_controllers_msgs::srv::ChangeKinematics_Request>::value &&
    has_fixed_size<robotnik_controllers_msgs::srv::ChangeKinematics_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::ChangeKinematics>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_controllers_msgs::srv::ChangeKinematics_Request>::value &&
    has_bounded_size<robotnik_controllers_msgs::srv::ChangeKinematics_Response>::value
  >
{
};

template<>
struct is_service<robotnik_controllers_msgs::srv::ChangeKinematics>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_controllers_msgs::srv::ChangeKinematics_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_controllers_msgs::srv::ChangeKinematics_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__CHANGE_KINEMATICS__TRAITS_HPP_
