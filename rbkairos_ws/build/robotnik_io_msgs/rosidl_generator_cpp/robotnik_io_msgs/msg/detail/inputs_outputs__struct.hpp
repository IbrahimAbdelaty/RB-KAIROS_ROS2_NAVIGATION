// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_io_msgs:msg/InputsOutputs.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__STRUCT_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'digital_inputs'
// Member 'digital_outputs'
#include "robotnik_io_msgs/msg/detail/digital_io__struct.hpp"
// Member 'analog_inputs'
// Member 'analog_outputs'
#include "robotnik_io_msgs/msg/detail/analog_io__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_io_msgs__msg__InputsOutputs __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_io_msgs__msg__InputsOutputs __declspec(deprecated)
#endif

namespace robotnik_io_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InputsOutputs_
{
  using Type = InputsOutputs_<ContainerAllocator>;

  explicit InputsOutputs_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit InputsOutputs_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _digital_inputs_type =
    std::vector<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>>;
  _digital_inputs_type digital_inputs;
  using _digital_outputs_type =
    std::vector<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>>;
  _digital_outputs_type digital_outputs;
  using _analog_inputs_type =
    std::vector<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>>>;
  _analog_inputs_type analog_inputs;
  using _analog_outputs_type =
    std::vector<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>>>;
  _analog_outputs_type analog_outputs;

  // setters for named parameter idiom
  Type & set__digital_inputs(
    const std::vector<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>> & _arg)
  {
    this->digital_inputs = _arg;
    return *this;
  }
  Type & set__digital_outputs(
    const std::vector<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>> & _arg)
  {
    this->digital_outputs = _arg;
    return *this;
  }
  Type & set__analog_inputs(
    const std::vector<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>>> & _arg)
  {
    this->analog_inputs = _arg;
    return *this;
  }
  Type & set__analog_outputs(
    const std::vector<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>>> & _arg)
  {
    this->analog_outputs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_io_msgs__msg__InputsOutputs
    std::shared_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_io_msgs__msg__InputsOutputs
    std::shared_ptr<robotnik_io_msgs::msg::InputsOutputs_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InputsOutputs_ & other) const
  {
    if (this->digital_inputs != other.digital_inputs) {
      return false;
    }
    if (this->digital_outputs != other.digital_outputs) {
      return false;
    }
    if (this->analog_inputs != other.analog_inputs) {
      return false;
    }
    if (this->analog_outputs != other.analog_outputs) {
      return false;
    }
    return true;
  }
  bool operator!=(const InputsOutputs_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InputsOutputs_

// alias to use template instance with default allocator
using InputsOutputs =
  robotnik_io_msgs::msg::InputsOutputs_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__STRUCT_HPP_
