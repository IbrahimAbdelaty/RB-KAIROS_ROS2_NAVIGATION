// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__STRUCT_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'registers'
#include "robotnik_io_msgs/msg/detail/register__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_io_msgs__msg__RegisterArray __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_io_msgs__msg__RegisterArray __declspec(deprecated)
#endif

namespace robotnik_io_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RegisterArray_
{
  using Type = RegisterArray_<ContainerAllocator>;

  explicit RegisterArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RegisterArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _registers_type =
    std::vector<robotnik_io_msgs::msg::Register_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::Register_<ContainerAllocator>>>;
  _registers_type registers;

  // setters for named parameter idiom
  Type & set__registers(
    const std::vector<robotnik_io_msgs::msg::Register_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::Register_<ContainerAllocator>>> & _arg)
  {
    this->registers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_io_msgs__msg__RegisterArray
    std::shared_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_io_msgs__msg__RegisterArray
    std::shared_ptr<robotnik_io_msgs::msg::RegisterArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RegisterArray_ & other) const
  {
    if (this->registers != other.registers) {
      return false;
    }
    return true;
  }
  bool operator!=(const RegisterArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RegisterArray_

// alias to use template instance with default allocator
using RegisterArray =
  robotnik_io_msgs::msg::RegisterArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__STRUCT_HPP_
