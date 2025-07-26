// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_controllers_msgs:msg/ControllerState.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__BUILDER_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_controllers_msgs/msg/detail/controller_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_controllers_msgs
{

namespace msg
{

namespace builder
{

class Init_ControllerState_state_number
{
public:
  explicit Init_ControllerState_state_number(::robotnik_controllers_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  ::robotnik_controllers_msgs::msg::ControllerState state_number(::robotnik_controllers_msgs::msg::ControllerState::_state_number_type arg)
  {
    msg_.state_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_controllers_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_state
{
public:
  explicit Init_ControllerState_state(::robotnik_controllers_msgs::msg::ControllerState & msg)
  : msg_(msg)
  {}
  Init_ControllerState_state_number state(::robotnik_controllers_msgs::msg::ControllerState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_ControllerState_state_number(msg_);
  }

private:
  ::robotnik_controllers_msgs::msg::ControllerState msg_;
};

class Init_ControllerState_header
{
public:
  Init_ControllerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControllerState_state header(::robotnik_controllers_msgs::msg::ControllerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ControllerState_state(msg_);
  }

private:
  ::robotnik_controllers_msgs::msg::ControllerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_controllers_msgs::msg::ControllerState>()
{
  return robotnik_controllers_msgs::msg::builder::Init_ControllerState_header();
}

}  // namespace robotnik_controllers_msgs

#endif  // ROBOTNIK_CONTROLLERS_MSGS__MSG__DETAIL__CONTROLLER_STATE__BUILDER_HPP_
