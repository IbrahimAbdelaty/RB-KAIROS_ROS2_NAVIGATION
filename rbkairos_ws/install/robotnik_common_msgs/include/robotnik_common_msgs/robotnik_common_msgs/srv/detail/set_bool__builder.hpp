// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_common_msgs:srv/SetBool.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_BOOL__BUILDER_HPP_
#define ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_BOOL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_common_msgs/srv/detail/set_bool__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_common_msgs
{

namespace srv
{

namespace builder
{

class Init_SetBool_Request_data
{
public:
  Init_SetBool_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_common_msgs::srv::SetBool_Request data(::robotnik_common_msgs::srv::SetBool_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_common_msgs::srv::SetBool_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_common_msgs::srv::SetBool_Request>()
{
  return robotnik_common_msgs::srv::builder::Init_SetBool_Request_data();
}

}  // namespace robotnik_common_msgs


namespace robotnik_common_msgs
{

namespace srv
{

namespace builder
{

class Init_SetBool_Response_response
{
public:
  Init_SetBool_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_common_msgs::srv::SetBool_Response response(::robotnik_common_msgs::srv::SetBool_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_common_msgs::srv::SetBool_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_common_msgs::srv::SetBool_Response>()
{
  return robotnik_common_msgs::srv::builder::Init_SetBool_Response_response();
}

}  // namespace robotnik_common_msgs

#endif  // ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_BOOL__BUILDER_HPP_
