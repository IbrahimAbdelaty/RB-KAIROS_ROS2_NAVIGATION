// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_battery_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_battery_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_battery_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryStatus_cell_voltages
{
public:
  explicit Init_BatteryStatus_cell_voltages(::robotnik_battery_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  ::robotnik_battery_msgs::msg::BatteryStatus cell_voltages(::robotnik_battery_msgs::msg::BatteryStatus::_cell_voltages_type arg)
  {
    msg_.cell_voltages = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_is_charging
{
public:
  explicit Init_BatteryStatus_is_charging(::robotnik_battery_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_cell_voltages is_charging(::robotnik_battery_msgs::msg::BatteryStatus::_is_charging_type arg)
  {
    msg_.is_charging = std::move(arg);
    return Init_BatteryStatus_cell_voltages(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_time_charging
{
public:
  explicit Init_BatteryStatus_time_charging(::robotnik_battery_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_is_charging time_charging(::robotnik_battery_msgs::msg::BatteryStatus::_time_charging_type arg)
  {
    msg_.time_charging = std::move(arg);
    return Init_BatteryStatus_is_charging(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_time_remaining
{
public:
  explicit Init_BatteryStatus_time_remaining(::robotnik_battery_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_time_charging time_remaining(::robotnik_battery_msgs::msg::BatteryStatus::_time_remaining_type arg)
  {
    msg_.time_remaining = std::move(arg);
    return Init_BatteryStatus_time_charging(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_level
{
public:
  explicit Init_BatteryStatus_level(::robotnik_battery_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_time_remaining level(::robotnik_battery_msgs::msg::BatteryStatus::_level_type arg)
  {
    msg_.level = std::move(arg);
    return Init_BatteryStatus_time_remaining(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_current
{
public:
  explicit Init_BatteryStatus_current(::robotnik_battery_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_level current(::robotnik_battery_msgs::msg::BatteryStatus::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_BatteryStatus_level(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_voltage
{
public:
  Init_BatteryStatus_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryStatus_current voltage(::robotnik_battery_msgs::msg::BatteryStatus::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BatteryStatus_current(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::BatteryStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_battery_msgs::msg::BatteryStatus>()
{
  return robotnik_battery_msgs::msg::builder::Init_BatteryStatus_voltage();
}

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
