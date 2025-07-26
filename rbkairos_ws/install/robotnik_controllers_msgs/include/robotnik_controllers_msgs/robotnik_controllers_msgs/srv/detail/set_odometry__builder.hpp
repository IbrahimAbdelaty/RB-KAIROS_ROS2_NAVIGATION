// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_controllers_msgs:srv/SetOdometry.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__BUILDER_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_controllers_msgs/srv/detail/set_odometry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_controllers_msgs
{

namespace srv
{

namespace builder
{

class Init_SetOdometry_Request_yaw
{
public:
  explicit Init_SetOdometry_Request_yaw(::robotnik_controllers_msgs::srv::SetOdometry_Request & msg)
  : msg_(msg)
  {}
  ::robotnik_controllers_msgs::srv::SetOdometry_Request yaw(::robotnik_controllers_msgs::srv::SetOdometry_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_controllers_msgs::srv::SetOdometry_Request msg_;
};

class Init_SetOdometry_Request_y
{
public:
  explicit Init_SetOdometry_Request_y(::robotnik_controllers_msgs::srv::SetOdometry_Request & msg)
  : msg_(msg)
  {}
  Init_SetOdometry_Request_yaw y(::robotnik_controllers_msgs::srv::SetOdometry_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SetOdometry_Request_yaw(msg_);
  }

private:
  ::robotnik_controllers_msgs::srv::SetOdometry_Request msg_;
};

class Init_SetOdometry_Request_x
{
public:
  Init_SetOdometry_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetOdometry_Request_y x(::robotnik_controllers_msgs::srv::SetOdometry_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetOdometry_Request_y(msg_);
  }

private:
  ::robotnik_controllers_msgs::srv::SetOdometry_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_controllers_msgs::srv::SetOdometry_Request>()
{
  return robotnik_controllers_msgs::srv::builder::Init_SetOdometry_Request_x();
}

}  // namespace robotnik_controllers_msgs


namespace robotnik_controllers_msgs
{

namespace srv
{

namespace builder
{

class Init_SetOdometry_Response_response
{
public:
  Init_SetOdometry_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_controllers_msgs::srv::SetOdometry_Response response(::robotnik_controllers_msgs::srv::SetOdometry_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_controllers_msgs::srv::SetOdometry_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_controllers_msgs::srv::SetOdometry_Response>()
{
  return robotnik_controllers_msgs::srv::builder::Init_SetOdometry_Response_response();
}

}  // namespace robotnik_controllers_msgs

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__SET_ODOMETRY__BUILDER_HPP_
