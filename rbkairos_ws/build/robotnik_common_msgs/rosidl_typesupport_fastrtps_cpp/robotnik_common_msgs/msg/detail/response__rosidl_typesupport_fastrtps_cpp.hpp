// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from robotnik_common_msgs:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robotnik_common_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "robotnik_common_msgs/msg/detail/response__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace robotnik_common_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotnik_common_msgs
cdr_serialize(
  const robotnik_common_msgs::msg::Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotnik_common_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robotnik_common_msgs::msg::Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotnik_common_msgs
get_serialized_size(
  const robotnik_common_msgs::msg::Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotnik_common_msgs
max_serialized_size_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robotnik_common_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robotnik_common_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robotnik_common_msgs, msg, Response)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
