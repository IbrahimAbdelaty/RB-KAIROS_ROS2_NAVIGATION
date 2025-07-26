// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_supervisor_msgs:srv/GetModuleList.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__TRAITS_HPP_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_supervisor_msgs/srv/detail/get_module_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_supervisor_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetModuleList_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: stack
  {
    out << "stack: ";
    rosidl_generator_traits::value_to_yaml(msg.stack, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetModuleList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stack
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stack: ";
    rosidl_generator_traits::value_to_yaml(msg.stack, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetModuleList_Request & msg, bool use_flow_style = false)
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
  const robotnik_supervisor_msgs::srv::GetModuleList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_supervisor_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_supervisor_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_supervisor_msgs::srv::GetModuleList_Request & msg)
{
  return robotnik_supervisor_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_supervisor_msgs::srv::GetModuleList_Request>()
{
  return "robotnik_supervisor_msgs::srv::GetModuleList_Request";
}

template<>
inline const char * name<robotnik_supervisor_msgs::srv::GetModuleList_Request>()
{
  return "robotnik_supervisor_msgs/srv/GetModuleList_Request";
}

template<>
struct has_fixed_size<robotnik_supervisor_msgs::srv::GetModuleList_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_supervisor_msgs::srv::GetModuleList_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_supervisor_msgs::srv::GetModuleList_Request>
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
  const GetModuleList_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    to_flow_style_yaml(msg.response, out);
    out << ", ";
  }

  // member: modules
  {
    if (msg.modules.size() == 0) {
      out << "modules: []";
    } else {
      out << "modules: [";
      size_t pending_items = msg.modules.size();
      for (auto item : msg.modules) {
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
  const GetModuleList_Response & msg,
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

  // member: modules
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.modules.size() == 0) {
      out << "modules: []\n";
    } else {
      out << "modules:\n";
      for (auto item : msg.modules) {
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

inline std::string to_yaml(const GetModuleList_Response & msg, bool use_flow_style = false)
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
  const robotnik_supervisor_msgs::srv::GetModuleList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_supervisor_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_supervisor_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_supervisor_msgs::srv::GetModuleList_Response & msg)
{
  return robotnik_supervisor_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_supervisor_msgs::srv::GetModuleList_Response>()
{
  return "robotnik_supervisor_msgs::srv::GetModuleList_Response";
}

template<>
inline const char * name<robotnik_supervisor_msgs::srv::GetModuleList_Response>()
{
  return "robotnik_supervisor_msgs/srv/GetModuleList_Response";
}

template<>
struct has_fixed_size<robotnik_supervisor_msgs::srv::GetModuleList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_supervisor_msgs::srv::GetModuleList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_supervisor_msgs::srv::GetModuleList_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_supervisor_msgs::srv::GetModuleList>()
{
  return "robotnik_supervisor_msgs::srv::GetModuleList";
}

template<>
inline const char * name<robotnik_supervisor_msgs::srv::GetModuleList>()
{
  return "robotnik_supervisor_msgs/srv/GetModuleList";
}

template<>
struct has_fixed_size<robotnik_supervisor_msgs::srv::GetModuleList>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_supervisor_msgs::srv::GetModuleList_Request>::value &&
    has_fixed_size<robotnik_supervisor_msgs::srv::GetModuleList_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_supervisor_msgs::srv::GetModuleList>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_supervisor_msgs::srv::GetModuleList_Request>::value &&
    has_bounded_size<robotnik_supervisor_msgs::srv::GetModuleList_Response>::value
  >
{
};

template<>
struct is_service<robotnik_supervisor_msgs::srv::GetModuleList>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_supervisor_msgs::srv::GetModuleList_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_supervisor_msgs::srv::GetModuleList_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__TRAITS_HPP_
