// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_perception_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
#define ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_perception_msgs/msg/detail/detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_perception_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionArray_objects
{
public:
  explicit Init_DetectionArray_objects(::robotnik_perception_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  ::robotnik_perception_msgs::msg::DetectionArray objects(::robotnik_perception_msgs::msg::DetectionArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_perception_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_header
{
public:
  Init_DetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionArray_objects header(::robotnik_perception_msgs::msg::DetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectionArray_objects(msg_);
  }

private:
  ::robotnik_perception_msgs::msg::DetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_perception_msgs::msg::DetectionArray>()
{
  return robotnik_perception_msgs::msg::builder::Init_DetectionArray_header();
}

}  // namespace robotnik_perception_msgs

#endif  // ROBOTNIK_PERCEPTION_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
