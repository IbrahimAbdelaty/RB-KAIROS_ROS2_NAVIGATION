// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_io_msgs:msg/Register.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__STRUCT_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_io_msgs__msg__Register __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_io_msgs__msg__Register __declspec(deprecated)
#endif

namespace robotnik_io_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Register_
{
  using Type = Register_<ContainerAllocator>;

  explicit Register_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address = 0l;
      this->value = 0l;
    }
  }

  explicit Register_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->address = 0l;
      this->value = 0l;
    }
  }

  // field types and members
  using _address_type =
    int32_t;
  _address_type address;
  using _value_type =
    int32_t;
  _value_type value;

  // setters for named parameter idiom
  Type & set__address(
    const int32_t & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__value(
    const int32_t & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_io_msgs::msg::Register_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_io_msgs::msg::Register_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::Register_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::Register_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_io_msgs__msg__Register
    std::shared_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_io_msgs__msg__Register
    std::shared_ptr<robotnik_io_msgs::msg::Register_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Register_ & other) const
  {
    if (this->address != other.address) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const Register_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Register_

// alias to use template instance with default allocator
using Register =
  robotnik_io_msgs::msg::Register_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__STRUCT_HPP_
