// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_common_msgs:msg/Response.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__STRUCT_HPP_
#define ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_common_msgs__msg__Response __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_common_msgs__msg__Response __declspec(deprecated)
#endif

namespace robotnik_common_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Response_
{
  using Type = Response_<ContainerAllocator>;

  explicit Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_common_msgs::msg::Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_common_msgs::msg::Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_common_msgs::msg::Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_common_msgs::msg::Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_common_msgs__msg__Response
    std::shared_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_common_msgs__msg__Response
    std::shared_ptr<robotnik_common_msgs::msg::Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Response_

// alias to use template instance with default allocator
using Response =
  robotnik_common_msgs::msg::Response_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robotnik_common_msgs

#endif  // ROBOTNIK_COMMON_MSGS__MSG__DETAIL__RESPONSE__STRUCT_HPP_
