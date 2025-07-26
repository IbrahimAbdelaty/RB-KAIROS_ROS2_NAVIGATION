// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/msg/detail/register_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace msg
{

namespace builder
{

class Init_RegisterArray_registers
{
public:
  Init_RegisterArray_registers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_io_msgs::msg::RegisterArray registers(::robotnik_io_msgs::msg::RegisterArray::_registers_type arg)
  {
    msg_.registers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::msg::RegisterArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::msg::RegisterArray>()
{
  return robotnik_io_msgs::msg::builder::Init_RegisterArray_registers();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__BUILDER_HPP_
