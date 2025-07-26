// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_controllers_msgs:srv/AvailableKinematics.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__TRAITS_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_controllers_msgs/srv/detail/available_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_controllers_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AvailableKinematics_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AvailableKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AvailableKinematics_Request & msg, bool use_flow_style = false)
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
  const robotnik_controllers_msgs::srv::AvailableKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::srv::AvailableKinematics_Request & msg)
{
  return robotnik_controllers_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::AvailableKinematics_Request>()
{
  return "robotnik_controllers_msgs::srv::AvailableKinematics_Request";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::AvailableKinematics_Request>()
{
  return "robotnik_controllers_msgs/srv/AvailableKinematics_Request";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::AvailableKinematics_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::AvailableKinematics_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robotnik_controllers_msgs::srv::AvailableKinematics_Request>
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
  const AvailableKinematics_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    to_flow_style_yaml(msg.response, out);
    out << ", ";
  }

  // member: kinematics
  {
    if (msg.kinematics.size() == 0) {
      out << "kinematics: []";
    } else {
      out << "kinematics: [";
      size_t pending_items = msg.kinematics.size();
      for (auto item : msg.kinematics) {
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
  const AvailableKinematics_Response & msg,
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

  // member: kinematics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.kinematics.size() == 0) {
      out << "kinematics: []\n";
    } else {
      out << "kinematics:\n";
      for (auto item : msg.kinematics) {
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

inline std::string to_yaml(const AvailableKinematics_Response & msg, bool use_flow_style = false)
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
  const robotnik_controllers_msgs::srv::AvailableKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_controllers_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_controllers_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_controllers_msgs::srv::AvailableKinematics_Response & msg)
{
  return robotnik_controllers_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::AvailableKinematics_Response>()
{
  return "robotnik_controllers_msgs::srv::AvailableKinematics_Response";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::AvailableKinematics_Response>()
{
  return "robotnik_controllers_msgs/srv/AvailableKinematics_Response";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::AvailableKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::AvailableKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_controllers_msgs::srv::AvailableKinematics_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_controllers_msgs::srv::AvailableKinematics>()
{
  return "robotnik_controllers_msgs::srv::AvailableKinematics";
}

template<>
inline const char * name<robotnik_controllers_msgs::srv::AvailableKinematics>()
{
  return "robotnik_controllers_msgs/srv/AvailableKinematics";
}

template<>
struct has_fixed_size<robotnik_controllers_msgs::srv::AvailableKinematics>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_controllers_msgs::srv::AvailableKinematics_Request>::value &&
    has_fixed_size<robotnik_controllers_msgs::srv::AvailableKinematics_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_controllers_msgs::srv::AvailableKinematics>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_controllers_msgs::srv::AvailableKinematics_Request>::value &&
    has_bounded_size<robotnik_controllers_msgs::srv::AvailableKinematics_Response>::value
  >
{
};

template<>
struct is_service<robotnik_controllers_msgs::srv::AvailableKinematics>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_controllers_msgs::srv::AvailableKinematics_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_controllers_msgs::srv::AvailableKinematics_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__TRAITS_HPP_
