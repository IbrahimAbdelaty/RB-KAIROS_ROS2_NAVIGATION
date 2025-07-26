// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_io_msgs:srv/GetRegisters.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__TRAITS_HPP_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_io_msgs/srv/detail/get_registers__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robotnik_io_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetRegisters_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: addresses
  {
    if (msg.addresses.size() == 0) {
      out << "addresses: []";
    } else {
      out << "addresses: [";
      size_t pending_items = msg.addresses.size();
      for (auto item : msg.addresses) {
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
  const GetRegisters_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: addresses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.addresses.size() == 0) {
      out << "addresses: []\n";
    } else {
      out << "addresses:\n";
      for (auto item : msg.addresses) {
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

inline std::string to_yaml(const GetRegisters_Request & msg, bool use_flow_style = false)
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
  const robotnik_io_msgs::srv::GetRegisters_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::srv::GetRegisters_Request & msg)
{
  return robotnik_io_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::srv::GetRegisters_Request>()
{
  return "robotnik_io_msgs::srv::GetRegisters_Request";
}

template<>
inline const char * name<robotnik_io_msgs::srv::GetRegisters_Request>()
{
  return "robotnik_io_msgs/srv/GetRegisters_Request";
}

template<>
struct has_fixed_size<robotnik_io_msgs::srv::GetRegisters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_io_msgs::srv::GetRegisters_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_io_msgs::srv::GetRegisters_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__traits.hpp"
// Member 'registers'
#include "robotnik_io_msgs/msg/detail/register__traits.hpp"

namespace robotnik_io_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetRegisters_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    to_flow_style_yaml(msg.response, out);
    out << ", ";
  }

  // member: registers
  {
    if (msg.registers.size() == 0) {
      out << "registers: []";
    } else {
      out << "registers: [";
      size_t pending_items = msg.registers.size();
      for (auto item : msg.registers) {
        to_flow_style_yaml(item, out);
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
  const GetRegisters_Response & msg,
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

  // member: registers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.registers.size() == 0) {
      out << "registers: []\n";
    } else {
      out << "registers:\n";
      for (auto item : msg.registers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetRegisters_Response & msg, bool use_flow_style = false)
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
  const robotnik_io_msgs::srv::GetRegisters_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::srv::GetRegisters_Response & msg)
{
  return robotnik_io_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::srv::GetRegisters_Response>()
{
  return "robotnik_io_msgs::srv::GetRegisters_Response";
}

template<>
inline const char * name<robotnik_io_msgs::srv::GetRegisters_Response>()
{
  return "robotnik_io_msgs/srv/GetRegisters_Response";
}

template<>
struct has_fixed_size<robotnik_io_msgs::srv::GetRegisters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_io_msgs::srv::GetRegisters_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_io_msgs::srv::GetRegisters_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robotnik_io_msgs::srv::GetRegisters>()
{
  return "robotnik_io_msgs::srv::GetRegisters";
}

template<>
inline const char * name<robotnik_io_msgs::srv::GetRegisters>()
{
  return "robotnik_io_msgs/srv/GetRegisters";
}

template<>
struct has_fixed_size<robotnik_io_msgs::srv::GetRegisters>
  : std::integral_constant<
    bool,
    has_fixed_size<robotnik_io_msgs::srv::GetRegisters_Request>::value &&
    has_fixed_size<robotnik_io_msgs::srv::GetRegisters_Response>::value
  >
{
};

template<>
struct has_bounded_size<robotnik_io_msgs::srv::GetRegisters>
  : std::integral_constant<
    bool,
    has_bounded_size<robotnik_io_msgs::srv::GetRegisters_Request>::value &&
    has_bounded_size<robotnik_io_msgs::srv::GetRegisters_Response>::value
  >
{
};

template<>
struct is_service<robotnik_io_msgs::srv::GetRegisters>
  : std::true_type
{
};

template<>
struct is_service_request<robotnik_io_msgs::srv::GetRegisters_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robotnik_io_msgs::srv::GetRegisters_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__TRAITS_HPP_
