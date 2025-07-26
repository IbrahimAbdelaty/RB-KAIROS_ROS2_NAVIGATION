// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:srv/GetRegisters.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/srv/detail/get_registers__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace srv
{

namespace builder
{

class Init_GetRegisters_Request_addresses
{
public:
  Init_GetRegisters_Request_addresses()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_io_msgs::srv::GetRegisters_Request addresses(::robotnik_io_msgs::srv::GetRegisters_Request::_addresses_type arg)
  {
    msg_.addresses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::srv::GetRegisters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::srv::GetRegisters_Request>()
{
  return robotnik_io_msgs::srv::builder::Init_GetRegisters_Request_addresses();
}

}  // namespace robotnik_io_msgs


namespace robotnik_io_msgs
{

namespace srv
{

namespace builder
{

class Init_GetRegisters_Response_registers
{
public:
  explicit Init_GetRegisters_Response_registers(::robotnik_io_msgs::srv::GetRegisters_Response & msg)
  : msg_(msg)
  {}
  ::robotnik_io_msgs::srv::GetRegisters_Response registers(::robotnik_io_msgs::srv::GetRegisters_Response::_registers_type arg)
  {
    msg_.registers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::srv::GetRegisters_Response msg_;
};

class Init_GetRegisters_Response_response
{
public:
  Init_GetRegisters_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRegisters_Response_registers response(::robotnik_io_msgs::srv::GetRegisters_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_GetRegisters_Response_registers(msg_);
  }

private:
  ::robotnik_io_msgs::srv::GetRegisters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::srv::GetRegisters_Response>()
{
  return robotnik_io_msgs::srv::builder::Init_GetRegisters_Response_response();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__BUILDER_HPP_
