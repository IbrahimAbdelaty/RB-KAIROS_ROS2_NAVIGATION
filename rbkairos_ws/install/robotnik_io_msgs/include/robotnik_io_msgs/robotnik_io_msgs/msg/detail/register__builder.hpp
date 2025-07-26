// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:msg/Register.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/msg/detail/register__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace msg
{

namespace builder
{

class Init_Register_value
{
public:
  explicit Init_Register_value(::robotnik_io_msgs::msg::Register & msg)
  : msg_(msg)
  {}
  ::robotnik_io_msgs::msg::Register value(::robotnik_io_msgs::msg::Register::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::msg::Register msg_;
};

class Init_Register_address
{
public:
  Init_Register_address()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Register_value address(::robotnik_io_msgs::msg::Register::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_Register_value(msg_);
  }

private:
  ::robotnik_io_msgs::msg::Register msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::msg::Register>()
{
  return robotnik_io_msgs::msg::builder::Init_Register_address();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__BUILDER_HPP_
