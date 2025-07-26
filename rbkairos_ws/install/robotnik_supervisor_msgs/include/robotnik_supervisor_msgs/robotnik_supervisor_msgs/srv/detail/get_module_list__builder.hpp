// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_supervisor_msgs:srv/GetModuleList.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__BUILDER_HPP_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_supervisor_msgs/srv/detail/get_module_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace builder
{

class Init_GetModuleList_Request_stack
{
public:
  Init_GetModuleList_Request_stack()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_supervisor_msgs::srv::GetModuleList_Request stack(::robotnik_supervisor_msgs::srv::GetModuleList_Request::_stack_type arg)
  {
    msg_.stack = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::GetModuleList_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_supervisor_msgs::srv::GetModuleList_Request>()
{
  return robotnik_supervisor_msgs::srv::builder::Init_GetModuleList_Request_stack();
}

}  // namespace robotnik_supervisor_msgs


namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace builder
{

class Init_GetModuleList_Response_modules
{
public:
  explicit Init_GetModuleList_Response_modules(::robotnik_supervisor_msgs::srv::GetModuleList_Response & msg)
  : msg_(msg)
  {}
  ::robotnik_supervisor_msgs::srv::GetModuleList_Response modules(::robotnik_supervisor_msgs::srv::GetModuleList_Response::_modules_type arg)
  {
    msg_.modules = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::GetModuleList_Response msg_;
};

class Init_GetModuleList_Response_response
{
public:
  Init_GetModuleList_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetModuleList_Response_modules response(::robotnik_supervisor_msgs::srv::GetModuleList_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_GetModuleList_Response_modules(msg_);
  }

private:
  ::robotnik_supervisor_msgs::srv::GetModuleList_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_supervisor_msgs::srv::GetModuleList_Response>()
{
  return robotnik_supervisor_msgs::srv::builder::Init_GetModuleList_Response_response();
}

}  // namespace robotnik_supervisor_msgs

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_MODULE_LIST__BUILDER_HPP_
