// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_supervisor_msgs:srv/StopModule.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__STOP_MODULE__BUILDER_HPP_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__STOP_MODULE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_supervisor_msgs/srv/detail/stop_module__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace builder
{

class Init_StopModule_Request_module
{
public:
  explicit Init_StopModule_Request_module(::robotnik_supervisor_msgs::srv::StopModule_Request & msg)
  : msg_(msg)
  {}
  ::robotnik_supervisor_msgs::srv::StopModule_Request module(::robotnik_supervisor_msgs::srv::StopModule_Request::_module_type arg)
  {
    msg_.module = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::StopModule_Request msg_;
};

class Init_StopModule_Request_stack
{
public:
  Init_StopModule_Request_stack()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopModule_Request_module stack(::robotnik_supervisor_msgs::srv::StopModule_Request::_stack_type arg)
  {
    msg_.stack = std::move(arg);
    return Init_StopModule_Request_module(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::StopModule_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_supervisor_msgs::srv::StopModule_Request>()
{
  return robotnik_supervisor_msgs::srv::builder::Init_StopModule_Request_stack();
}

}  // namespace robotnik_supervisor_msgs


namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace builder
{

class Init_StopModule_Response_response
{
public:
  Init_StopModule_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_supervisor_msgs::srv::StopModule_Response response(::robotnik_supervisor_msgs::srv::StopModule_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::StopModule_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_supervisor_msgs::srv::StopModule_Response>()
{
  return robotnik_supervisor_msgs::srv::builder::Init_StopModule_Response_response();
}

}  // namespace robotnik_supervisor_msgs

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__STOP_MODULE__BUILDER_HPP_
