// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_io_msgs:msg/DigitalIO.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__STRUCT_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_io_msgs__msg__DigitalIO __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_io_msgs__msg__DigitalIO __declspec(deprecated)
#endif

namespace robotnik_io_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DigitalIO_
{
  using Type = DigitalIO_<ContainerAllocator>;

  explicit DigitalIO_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->name = "";
      this->value = false;
    }
  }

  explicit DigitalIO_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->name = "";
      this->value = false;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _value_type =
    bool;
  _value_type value;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__value(
    const bool & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_io_msgs__msg__DigitalIO
    std::shared_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_io_msgs__msg__DigitalIO
    std::shared_ptr<robotnik_io_msgs::msg::DigitalIO_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DigitalIO_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const DigitalIO_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DigitalIO_

// alias to use template instance with default allocator
using DigitalIO =
  robotnik_io_msgs::msg::DigitalIO_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__DIGITAL_IO__STRUCT_HPP_
