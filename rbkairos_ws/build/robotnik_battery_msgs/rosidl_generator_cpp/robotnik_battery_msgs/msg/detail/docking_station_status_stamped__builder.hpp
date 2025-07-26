// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_battery_msgs:msg/DockingStationStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS_STAMPED__BUILDER_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_battery_msgs/msg/detail/docking_station_status_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_battery_msgs
{

namespace msg
{

namespace builder
{

class Init_DockingStationStatusStamped_status
{
public:
  explicit Init_DockingStationStatusStamped_status(::robotnik_battery_msgs::msg::DockingStationStatusStamped & msg)
  : msg_(msg)
  {}
  ::robotnik_battery_msgs::msg::DockingStationStatusStamped status(::robotnik_battery_msgs::msg::DockingStationStatusStamped::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::DockingStationStatusStamped msg_;
};

class Init_DockingStationStatusStamped_header
{
public:
  Init_DockingStationStatusStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DockingStationStatusStamped_status header(::robotnik_battery_msgs::msg::DockingStationStatusStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DockingStationStatusStamped_status(msg_);
  }

private:
  ::robotnik_battery_msgs::msg::DockingStationStatusStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_battery_msgs::msg::DockingStationStatusStamped>()
{
  return robotnik_battery_msgs::msg::builder::Init_DockingStationStatusStamped_header();
}

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS_STAMPED__BUILDER_HPP_
