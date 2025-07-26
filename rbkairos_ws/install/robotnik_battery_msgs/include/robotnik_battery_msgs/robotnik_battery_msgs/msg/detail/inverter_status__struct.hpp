// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_battery_msgs:msg/InverterStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__STRUCT_HPP_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_battery_msgs__msg__InverterStatus __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_battery_msgs__msg__InverterStatus __declspec(deprecated)
#endif

namespace robotnik_battery_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InverterStatus_
{
  using Type = InverterStatus_<ContainerAllocator>;

  explicit InverterStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ac_voltage = 0.0f;
      this->dc_voltage = 0.0f;
      this->load = 0.0f;
      this->percentage = 0.0f;
      this->temperature = 0.0f;
      this->on = false;
      this->serial_number = "";
    }
  }

  explicit InverterStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : serial_number(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ac_voltage = 0.0f;
      this->dc_voltage = 0.0f;
      this->load = 0.0f;
      this->percentage = 0.0f;
      this->temperature = 0.0f;
      this->on = false;
      this->serial_number = "";
    }
  }

  // field types and members
  using _ac_voltage_type =
    float;
  _ac_voltage_type ac_voltage;
  using _dc_voltage_type =
    float;
  _dc_voltage_type dc_voltage;
  using _load_type =
    float;
  _load_type load;
  using _percentage_type =
    float;
  _percentage_type percentage;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _on_type =
    bool;
  _on_type on;
  using _serial_number_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _serial_number_type serial_number;

  // setters for named parameter idiom
  Type & set__ac_voltage(
    const float & _arg)
  {
    this->ac_voltage = _arg;
    return *this;
  }
  Type & set__dc_voltage(
    const float & _arg)
  {
    this->dc_voltage = _arg;
    return *this;
  }
  Type & set__load(
    const float & _arg)
  {
    this->load = _arg;
    return *this;
  }
  Type & set__percentage(
    const float & _arg)
  {
    this->percentage = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__on(
    const bool & _arg)
  {
    this->on = _arg;
    return *this;
  }
  Type & set__serial_number(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->serial_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_battery_msgs__msg__InverterStatus
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_battery_msgs__msg__InverterStatus
    std::shared_ptr<robotnik_battery_msgs::msg::InverterStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InverterStatus_ & other) const
  {
    if (this->ac_voltage != other.ac_voltage) {
      return false;
    }
    if (this->dc_voltage != other.dc_voltage) {
      return false;
    }
    if (this->load != other.load) {
      return false;
    }
    if (this->percentage != other.percentage) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->on != other.on) {
      return false;
    }
    if (this->serial_number != other.serial_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const InverterStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InverterStatus_

// alias to use template instance with default allocator
using InverterStatus =
  robotnik_battery_msgs::msg::InverterStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_battery_msgs

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__STRUCT_HPP_
