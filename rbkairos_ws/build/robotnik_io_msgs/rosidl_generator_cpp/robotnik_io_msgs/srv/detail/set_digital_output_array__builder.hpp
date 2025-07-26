// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:srv/SetDigitalOutputArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_DIGITAL_OUTPUT_ARRAY__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_DIGITAL_OUTPUT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/srv/detail/set_digital_output_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace srv
{

namespace builder
{

class Init_SetDigitalOutputArray_Request_output
{
public:
  Init_SetDigitalOutputArray_Request_output()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_io_msgs::srv::SetDigitalOutputArray_Request output(::robotnik_io_msgs::srv::SetDigitalOutputArray_Request::_output_type arg)
  {
    msg_.output = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::srv::SetDigitalOutputArray_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::srv::SetDigitalOutputArray_Request>()
{
  return robotnik_io_msgs::srv::builder::Init_SetDigitalOutputArray_Request_output();
}

}  // namespace robotnik_io_msgs


namespace robotnik_io_msgs
{

namespace srv
{

namespace builder
{

class Init_SetDigitalOutputArray_Response_response
{
public:
  Init_SetDigitalOutputArray_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_io_msgs::srv::SetDigitalOutputArray_Response response(::robotnik_io_msgs::srv::SetDigitalOutputArray_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::srv::SetDigitalOutputArray_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::srv::SetDigitalOutputArray_Response>()
{
  return robotnik_io_msgs::srv::builder::Init_SetDigitalOutputArray_Response_response();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_DIGITAL_OUTPUT_ARRAY__BUILDER_HPP_
