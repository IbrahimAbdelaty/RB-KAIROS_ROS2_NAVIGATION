// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_battery_msgs:msg/DockingStationStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__BUILDER_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_battery_msgs/msg/detail/docking_station_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_battery_msgs
{

namespace msg
{

namespace builder
{

class Init_DockingStationStatus_charger_relay_status
{
public:
  explicit Init_DockingStationStatus_charger_relay_status(::robotnik_battery_msgs::msg::DockingStationStatus & msg)
  : msg_(msg)
  {}
  ::robotnik_battery_msgs::msg::DockingStationStatus charger_relay_status(::robotnik_battery_msgs::msg::DockingStationStatus::_charger_relay_status_type arg)
  {
    msg_.charger_relay_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::DockingStationStatus msg_;
};

class Init_DockingStationStatus_contact_relay_status
{
public:
  explicit Init_DockingStationStatus_contact_relay_status(::robotnik_battery_msgs::msg::DockingStationStatus & msg)
  : msg_(msg)
  {}
  Init_DockingStationStatus_charger_relay_status contact_relay_status(::robotnik_battery_msgs::msg::DockingStationStatus::_contact_relay_status_type arg)
  {
    msg_.contact_relay_status = std::move(arg);
    return Init_DockingStationStatus_charger_relay_status(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::DockingStationStatus msg_;
};

class Init_DockingStationStatus_operation_mode
{
public:
  Init_DockingStationStatus_operation_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DockingStationStatus_contact_relay_status operation_mode(::robotnik_battery_msgs::msg::DockingStationStatus::_operation_mode_type arg)
  {
    msg_.operation_mode = std::move(arg);
    return Init_DockingStationStatus_contact_relay_status(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::DockingStationStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_battery_msgs::msg::DockingStationStatus>()
{
  return robotnik_battery_msgs::msg::builder::Init_DockingStationStatus_operation_mode();
}

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__BUILDER_HPP_
