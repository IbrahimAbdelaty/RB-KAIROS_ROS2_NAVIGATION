// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_perception_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_perception_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_perception_msgs
{

namespace msg
{

namespace builder
{

class Init_Detection_object
{
public:
  explicit Init_Detection_object(::robotnik_perception_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  ::robotnik_perception_msgs::msg::Detection object(::robotnik_perception_msgs::msg::Detection::_object_type arg)
  {
    msg_.object = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_perception_msgs::msg::Detection msg_;
};

class Init_Detection_header
{
public:
  Init_Detection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_object header(::robotnik_perception_msgs::msg::Detection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Detection_object(msg_);
  }

private:
  ::robotnik_perception_msgs::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_perception_msgs::msg::Detection>()
{
  return robotnik_perception_msgs::msg::builder::Init_Detection_header();
}

}  // namespace robotnik_perception_msgs

#endif  // ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
