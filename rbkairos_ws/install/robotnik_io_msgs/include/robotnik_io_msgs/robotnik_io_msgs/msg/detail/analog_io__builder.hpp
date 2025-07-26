// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:msg/AnalogIO.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__ANALOG_IO__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__ANALOG_IO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/msg/detail/analog_io__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace msg
{

namespace builder
{

class Init_AnalogIO_value
{
public:
  explicit Init_AnalogIO_value(::robotnik_io_msgs::msg::AnalogIO & msg)
  : msg_(msg)
  {}
  ::robotnik_io_msgs::msg::AnalogIO value(::robotnik_io_msgs::msg::AnalogIO::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::msg::AnalogIO msg_;
};

class Init_AnalogIO_name
{
public:
  explicit Init_AnalogIO_name(::robotnik_io_msgs::msg::AnalogIO & msg)
  : msg_(msg)
  {}
  Init_AnalogIO_value name(::robotnik_io_msgs::msg::AnalogIO::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_AnalogIO_value(msg_);
  }

private:
  ::robotnik_io_msgs::msg::AnalogIO msg_;
};

class Init_AnalogIO_id
{
public:
  Init_AnalogIO_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AnalogIO_name id(::robotnik_io_msgs::msg::AnalogIO::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_AnalogIO_name(msg_);
  }

private:
  ::robotnik_io_msgs::msg::AnalogIO msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::msg::AnalogIO>()
{
  return robotnik_io_msgs::msg::builder::Init_AnalogIO_id();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__ANALOG_IO__BUILDER_HPP_
