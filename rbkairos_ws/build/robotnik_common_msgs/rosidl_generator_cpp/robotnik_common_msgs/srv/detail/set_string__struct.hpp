// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_common_msgs:srv/SetString.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_STRING__STRUCT_HPP_
#define ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_STRING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_common_msgs__srv__SetString_Request __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_common_msgs__srv__SetString_Request __declspec(deprecated)
#endif

namespace robotnik_common_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetString_Request_
{
  using Type = SetString_Request_<ContainerAllocator>;

  explicit SetString_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit SetString_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_common_msgs__srv__SetString_Request
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_common_msgs__srv__SetString_Request
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetString_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetString_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetString_Request_

// alias to use template instance with default allocator
using SetString_Request =
  robotnik_common_msgs::srv::SetString_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotnik_common_msgs


// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_common_msgs__srv__SetString_Response __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_common_msgs__srv__SetString_Response __declspec(deprecated)
#endif

namespace robotnik_common_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetString_Response_
{
  using Type = SetString_Response_<ContainerAllocator>;

  explicit SetString_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_init)
  {
    (void)_init;
  }

  explicit SetString_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _response_type =
    robotnik_common_msgs::msg::Response_<ContainerAllocator>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const robotnik_common_msgs::msg::Response_<ContainerAllocator> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_common_msgs__srv__SetString_Response
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_common_msgs__srv__SetString_Response
    std::shared_ptr<robotnik_common_msgs::srv::SetString_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetString_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetString_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetString_Response_

// alias to use template instance with default allocator
using SetString_Response =
  robotnik_common_msgs::srv::SetString_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotnik_common_msgs

namespace robotnik_common_msgs
{

namespace srv
{

struct SetString
{
  using Request = robotnik_common_msgs::srv::SetString_Request;
  using Response = robotnik_common_msgs::srv::SetString_Response;
};

}  // namespace srv

}  // namespace robotnik_common_msgs

#endif  // ROBOTNIK_COMMON_MSGS__SRV__DETAIL__SET_STRING__STRUCT_HPP_
