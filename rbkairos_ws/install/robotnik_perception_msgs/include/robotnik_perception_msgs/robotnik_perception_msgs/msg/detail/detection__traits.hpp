// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robotnik_perception_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robotnik_perception_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'object'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace robotnik_perception_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: object
  {
    out << "object: ";
    to_flow_style_yaml(msg.object, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection & msg,
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

  // member: object
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object:\n";
    to_block_style_yaml(msg.object, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
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

}  // namespace robotnik_perception_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robotnik_perception_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robotnik_perception_msgs::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  robotnik_perception_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robotnik_perception_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robotnik_perception_msgs::msg::Detection & msg)
{
  return robotnik_perception_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robotnik_perception_msgs::msg::Detection>()
{
  return "robotnik_perception_msgs::msg::Detection";
}

template<>
inline const char * name<robotnik_perception_msgs::msg::Detection>()
{
  return "robotnik_perception_msgs/msg/Detection";
}

template<>
struct has_fixed_size<robotnik_perception_msgs::msg::Detection>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<robotnik_perception_msgs::msg::Detection>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<robotnik_perception_msgs::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
