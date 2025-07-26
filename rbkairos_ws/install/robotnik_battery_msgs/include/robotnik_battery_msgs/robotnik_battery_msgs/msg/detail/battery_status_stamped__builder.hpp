// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_battery_msgs:msg/BatteryStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__BUILDER_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_battery_msgs/msg/detail/battery_status_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_battery_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryStatusStamped_status
{
public:
  explicit Init_BatteryStatusStamped_status(::robotnik_battery_msgs::msg::BatteryStatusStamped & msg)
  : msg_(msg)
  {}
  ::robotnik_battery_msgs::msg::BatteryStatusStamped status(::robotnik_battery_msgs::msg::BatteryStatusStamped::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatusStamped msg_;
};

class Init_BatteryStatusStamped_header
{
public:
  Init_BatteryStatusStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryStatusStamped_status header(::robotnik_battery_msgs::msg::BatteryStatusStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BatteryStatusStamped_status(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatusStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_battery_msgs::msg::BatteryStatusStamped>()
{
  return robotnik_battery_msgs::msg::builder::Init_BatteryStatusStamped_header();
}

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__BUILDER_HPP_
