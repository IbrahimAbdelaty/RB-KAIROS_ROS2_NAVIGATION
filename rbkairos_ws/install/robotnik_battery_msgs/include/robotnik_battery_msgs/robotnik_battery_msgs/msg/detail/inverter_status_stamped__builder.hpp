// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_battery_msgs:msg/InverterStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__BUILDER_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_battery_msgs/msg/detail/inverter_status_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_battery_msgs
{

namespace msg
{

namespace builder
{

class Init_InverterStatusStamped_status
{
public:
  explicit Init_InverterStatusStamped_status(::robotnik_battery_msgs::msg::InverterStatusStamped & msg)
  : msg_(msg)
  {}
  ::robotnik_battery_msgs::msg::InverterStatusStamped status(::robotnik_battery_msgs::msg::InverterStatusStamped::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatusStamped msg_;
};

class Init_InverterStatusStamped_header
{
public:
  Init_InverterStatusStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InverterStatusStamped_status header(::robotnik_battery_msgs::msg::InverterStatusStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_InverterStatusStamped_status(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatusStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_battery_msgs::msg::InverterStatusStamped>()
{
  return robotnik_battery_msgs::msg::builder::Init_InverterStatusStamped_header();
}

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__BUILDER_HPP_
