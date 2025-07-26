// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_io_msgs:msg/InputsOutputs.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__TRAITS_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_io_msgs/msg/detail/inputs_outputs__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'digital_inputs'
// Member 'digital_outputs'
#include "robotnik_io_msgs/msg/detail/digital_io__traits.hpp"
// Member 'analog_inputs'
// Member 'analog_outputs'
#include "robotnik_io_msgs/msg/detail/analog_io__traits.hpp"

namespace robotnik_io_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const InputsOutputs & msg,
  std::ostream & out)
{
  out << "{";
  // member: digital_inputs
  {
    if (msg.digital_inputs.size() == 0) {
      out << "digital_inputs: []";
    } else {
      out << "digital_inputs: [";
      size_t pending_items = msg.digital_inputs.size();
      for (auto item : msg.digital_inputs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: digital_outputs
  {
    if (msg.digital_outputs.size() == 0) {
      out << "digital_outputs: []";
    } else {
      out << "digital_outputs: [";
      size_t pending_items = msg.digital_outputs.size();
      for (auto item : msg.digital_outputs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: analog_inputs
  {
    if (msg.analog_inputs.size() == 0) {
      out << "analog_inputs: []";
    } else {
      out << "analog_inputs: [";
      size_t pending_items = msg.analog_inputs.size();
      for (auto item : msg.analog_inputs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: analog_outputs
  {
    if (msg.analog_outputs.size() == 0) {
      out << "analog_outputs: []";
    } else {
      out << "analog_outputs: [";
      size_t pending_items = msg.analog_outputs.size();
      for (auto item : msg.analog_outputs) {
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
  const InputsOutputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: digital_inputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.digital_inputs.size() == 0) {
      out << "digital_inputs: []\n";
    } else {
      out << "digital_inputs:\n";
      for (auto item : msg.digital_inputs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: digital_outputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.digital_outputs.size() == 0) {
      out << "digital_outputs: []\n";
    } else {
      out << "digital_outputs:\n";
      for (auto item : msg.digital_outputs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: analog_inputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.analog_inputs.size() == 0) {
      out << "analog_inputs: []\n";
    } else {
      out << "analog_inputs:\n";
      for (auto item : msg.analog_inputs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: analog_outputs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.analog_outputs.size() == 0) {
      out << "analog_outputs: []\n";
    } else {
      out << "analog_outputs:\n";
      for (auto item : msg.analog_outputs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InputsOutputs & msg, bool use_flow_style = false)
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

}  // namespace robotnik_io_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_io_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_io_msgs::msg::InputsOutputs & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_io_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_io_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_io_msgs::msg::InputsOutputs & msg)
{
  return robotnik_io_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_io_msgs::msg::InputsOutputs>()
{
  return "robotnik_io_msgs::msg::InputsOutputs";
}

template<>
inline const char * name<robotnik_io_msgs::msg::InputsOutputs>()
{
  return "robotnik_io_msgs/msg/InputsOutputs";
}

template<>
struct has_fixed_size<robotnik_io_msgs::msg::InputsOutputs>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robotnik_io_msgs::msg::InputsOutputs>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robotnik_io_msgs::msg::InputsOutputs>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__TRAITS_HPP_
