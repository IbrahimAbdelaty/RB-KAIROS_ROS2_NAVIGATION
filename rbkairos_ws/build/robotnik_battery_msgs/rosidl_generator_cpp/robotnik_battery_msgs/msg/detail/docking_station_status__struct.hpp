// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_battery_msgs:msg/DockingStationStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__STRUCT_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_battery_msgs__msg__DockingStationStatus __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_battery_msgs__msg__DockingStationStatus __declspec(deprecated)
#endif

namespace robotnik_battery_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DockingStationStatus_
{
  using Type = DockingStationStatus_<ContainerAllocator>;

  explicit DockingStationStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operation_mode = "";
      this->contact_relay_status = false;
      this->charger_relay_status = false;
    }
  }

  explicit DockingStationStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : operation_mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operation_mode = "";
      this->contact_relay_status = false;
      this->charger_relay_status = false;
    }
  }

  // field types and members
  using _operation_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _operation_mode_type operation_mode;
  using _contact_relay_status_type =
    bool;
  _contact_relay_status_type contact_relay_status;
  using _charger_relay_status_type =
    bool;
  _charger_relay_status_type charger_relay_status;

  // setters for named parameter idiom
  Type & set__operation_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->operation_mode = _arg;
    return *this;
  }
  Type & set__contact_relay_status(
    const bool & _arg)
  {
    this->contact_relay_status = _arg;
    return *this;
  }
  Type & set__charger_relay_status(
    const bool & _arg)
  {
    this->charger_relay_status = _arg;
    return *this;
  }

  // constant declarations
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MODE_DISABLED;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MODE_AUTO_HW;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MODE_AUTO_SW;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MODE_MANUAL_SW;

  // pointer types
  using RawPtr =
    robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_battery_msgs__msg__DockingStationStatus
    std::shared_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_battery_msgs__msg__DockingStationStatus
    std::shared_ptr<robotnik_battery_msgs::msg::DockingStationStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DockingStationStatus_ & other) const
  {
    if (this->operation_mode != other.operation_mode) {
      return false;
    }
    if (this->contact_relay_status != other.contact_relay_status) {
      return false;
    }
    if (this->charger_relay_status != other.charger_relay_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const DockingStationStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DockingStationStatus_

// alias to use template instance with default allocator
using DockingStationStatus =
  robotnik_battery_msgs::msg::DockingStationStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
DockingStationStatus_<ContainerAllocator>::MODE_DISABLED = "disabled";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
DockingStationStatus_<ContainerAllocator>::MODE_AUTO_HW = "automatic_hw";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
DockingStationStatus_<ContainerAllocator>::MODE_AUTO_SW = "automatic_sw";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
DockingStationStatus_<ContainerAllocator>::MODE_MANUAL_SW = "manual_sw";

}  // namespace msg

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__STRUCT_HPP_
