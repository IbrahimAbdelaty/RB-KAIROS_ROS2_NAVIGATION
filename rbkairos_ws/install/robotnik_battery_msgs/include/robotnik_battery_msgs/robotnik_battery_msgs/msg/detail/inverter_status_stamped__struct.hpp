// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_battery_msgs:msg/InverterStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__STRUCT_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'status'
#include "robotnik_battery_msgs/msg/detail/inverter_status__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_battery_msgs__msg__InverterStatusStamped __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_battery_msgs__msg__InverterStatusStamped __declspec(deprecated)
#endif

namespace robotnik_battery_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InverterStatusStamped_
{
  using Type = InverterStatusStamped_<ContainerAllocator>;

  explicit InverterStatusStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    status(_init)
  {
    (void)_init;
  }

  explicit InverterStatusStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    status(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_battery_msgs__msg__InverterStatusStamped
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_battery_msgs__msg__InverterStatusStamped
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatusStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InverterStatusStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const InverterStatusStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InverterStatusStamped_

// alias to use template instance with default allocator
using InverterStatusStamped =
  robotnik_battery_msgs::msg::InverterStatusStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS_STAMPED__STRUCT_HPP_
