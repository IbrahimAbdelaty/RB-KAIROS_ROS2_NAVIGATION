// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_battery_msgs:msg/InverterStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__BUILDER_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_battery_msgs/msg/detail/inverter_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_battery_msgs
{

namespace msg
{

namespace builder
{

class Init_InverterStatus_serial_number
{
public:
  explicit Init_InverterStatus_serial_number(::robotnik_battery_msgs::msg::InverterStatus & msg)
  : msg_(msg)
  {}
  ::robotnik_battery_msgs::msg::InverterStatus serial_number(::robotnik_battery_msgs::msg::InverterStatus::_serial_number_type arg)
  {
    msg_.serial_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

class Init_InverterStatus_on
{
public:
  explicit Init_InverterStatus_on(::robotnik_battery_msgs::msg::InverterStatus & msg)
  : msg_(msg)
  {}
  Init_InverterStatus_serial_number on(::robotnik_battery_msgs::msg::InverterStatus::_on_type arg)
  {
    msg_.on = std::move(arg);
    return Init_InverterStatus_serial_number(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

class Init_InverterStatus_temperature
{
public:
  explicit Init_InverterStatus_temperature(::robotnik_battery_msgs::msg::InverterStatus & msg)
  : msg_(msg)
  {}
  Init_InverterStatus_on temperature(::robotnik_battery_msgs::msg::InverterStatus::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_InverterStatus_on(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

class Init_InverterStatus_percentage
{
public:
  explicit Init_InverterStatus_percentage(::robotnik_battery_msgs::msg::InverterStatus & msg)
  : msg_(msg)
  {}
  Init_InverterStatus_temperature percentage(::robotnik_battery_msgs::msg::InverterStatus::_percentage_type arg)
  {
    msg_.percentage = std::move(arg);
    return Init_InverterStatus_temperature(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

class Init_InverterStatus_load
{
public:
  explicit Init_InverterStatus_load(::robotnik_battery_msgs::msg::InverterStatus & msg)
  : msg_(msg)
  {}
  Init_InverterStatus_percentage load(::robotnik_battery_msgs::msg::InverterStatus::_load_type arg)
  {
    msg_.load = std::move(arg);
    return Init_InverterStatus_percentage(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

class Init_InverterStatus_dc_voltage
{
public:
  explicit Init_InverterStatus_dc_voltage(::robotnik_battery_msgs::msg::InverterStatus & msg)
  : msg_(msg)
  {}
  Init_InverterStatus_load dc_voltage(::robotnik_battery_msgs::msg::InverterStatus::_dc_voltage_type arg)
  {
    msg_.dc_voltage = std::move(arg);
    return Init_InverterStatus_load(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

class Init_InverterStatus_ac_voltage
{
public:
  Init_InverterStatus_ac_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InverterStatus_dc_voltage ac_voltage(::robotnik_battery_msgs::msg::InverterStatus::_ac_voltage_type arg)
  {
    msg_.ac_voltage = std::move(arg);
    return Init_InverterStatus_dc_voltage(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::InverterStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_battery_msgs::msg::InverterStatus>()
{
  return robotnik_battery_msgs::msg::builder::Init_InverterStatus_ac_voltage();
}

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__BUILDER_HPP_
