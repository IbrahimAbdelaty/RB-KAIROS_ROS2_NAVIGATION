// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:msg/DigitalIO.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/msg/detail/digital_io__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace msg
{

namespace builder
{

class Init_DigitalIO_value
{
public:
  explicit Init_DigitalIO_value(::robotnik_io_msgs::msg::DigitalIO & msg)
  : msg_(msg)
  {}
  ::robotnik_io_msgs::msg::DigitalIO value(::robotnik_io_msgs::msg::DigitalIO::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::msg::DigitalIO msg_;
};

class Init_DigitalIO_name
{
public:
  explicit Init_DigitalIO_name(::robotnik_io_msgs::msg::DigitalIO & msg)
  : msg_(msg)
  {}
  Init_DigitalIO_value name(::robotnik_io_msgs::msg::DigitalIO::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_DigitalIO_value(msg_);
  }

private:
  ::robotnik_io_msgs::msg::DigitalIO msg_;
};

class Init_DigitalIO_id
{
public:
  Init_DigitalIO_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DigitalIO_name id(::robotnik_io_msgs::msg::DigitalIO::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DigitalIO_name(msg_);
  }

private:
  ::robotnik_io_msgs::msg::DigitalIO msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::msg::DigitalIO>()
{
  return robotnik_io_msgs::msg::builder::Init_DigitalIO_id();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__BUILDER_HPP_
