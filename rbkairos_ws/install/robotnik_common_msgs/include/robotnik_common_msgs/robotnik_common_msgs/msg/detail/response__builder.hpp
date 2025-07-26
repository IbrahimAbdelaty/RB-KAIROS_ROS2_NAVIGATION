// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_common_msgs:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__BUILDER_HPP_
#define ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_common_msgs/msg/detail/response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_common_msgs
{

namespace msg
{

namespace builder
{

class Init_Response_message
{
public:
  explicit Init_Response_message(::robotnik_common_msgs::msg::Response & msg)
  : msg_(msg)
  {}
  ::robotnik_common_msgs::msg::Response message(::robotnik_common_msgs::msg::Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_common_msgs::msg::Response msg_;
};

class Init_Response_success
{
public:
  Init_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Response_message success(::robotnik_common_msgs::msg::Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Response_message(msg_);
  }

private:
  ::robotnik_common_msgs::msg::Response msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_common_msgs::msg::Response>()
{
  return robotnik_common_msgs::msg::builder::Init_Response_success();
}

}  // namespace robotnik_common_msgs

#endif  // ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__BUILDER_HPP_
