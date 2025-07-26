// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_supervisor_msgs:srv/RestartStack.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__RESTART_STACK__BUILDER_HPP_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__RESTART_STACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_supervisor_msgs/srv/detail/restart_stack__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace builder
{

class Init_RestartStack_Request_name
{
public:
  Init_RestartStack_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_supervisor_msgs::srv::RestartStack_Request name(::robotnik_supervisor_msgs::srv::RestartStack_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::RestartStack_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_supervisor_msgs::srv::RestartStack_Request>()
{
  return robotnik_supervisor_msgs::srv::builder::Init_RestartStack_Request_name();
}

}  // namespace robotnik_supervisor_msgs


namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace builder
{

class Init_RestartStack_Response_response
{
public:
  Init_RestartStack_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_supervisor_msgs::srv::RestartStack_Response response(::robotnik_supervisor_msgs::srv::RestartStack_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::RestartStack_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_supervisor_msgs::srv::RestartStack_Response>()
{
  return robotnik_supervisor_msgs::srv::builder::Init_RestartStack_Response_response();
}

}  // namespace robotnik_supervisor_msgs

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__RESTART_STACK__BUILDER_HPP_
