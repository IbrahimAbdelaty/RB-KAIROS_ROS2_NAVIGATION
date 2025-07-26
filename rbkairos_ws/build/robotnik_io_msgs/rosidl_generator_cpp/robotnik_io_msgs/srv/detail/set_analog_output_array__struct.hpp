// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_io_msgs:srv/SetAnalogOutputArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT_ARRAY__STRUCT_HPP_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'output'
#include "robotnik_io_msgs/msg/detail/analog_io__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Request __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Request __declspec(deprecated)
#endif

namespace robotnik_io_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetAnalogOutputArray_Request_
{
  using Type = SetAnalogOutputArray_Request_<ContainerAllocator>;

  explicit SetAnalogOutputArray_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SetAnalogOutputArray_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _output_type =
    std::vector<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>>>;
  _output_type output;

  // setters for named parameter idiom
  Type & set__output(
    const std::vector<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robotnik_io_msgs::msg::AnalogIO_<ContainerAllocator>>> & _arg)
  {
    this->output = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Request
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Request
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetAnalogOutputArray_Request_ & other) const
  {
    if (this->output != other.output) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetAnalogOutputArray_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetAnalogOutputArray_Request_

// alias to use template instance with default allocator
using SetAnalogOutputArray_Request =
  robotnik_io_msgs::srv::SetAnalogOutputArray_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotnik_io_msgs


// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Response __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Response __declspec(deprecated)
#endif

namespace robotnik_io_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetAnalogOutputArray_Response_
{
  using Type = SetAnalogOutputArray_Response_<ContainerAllocator>;

  explicit SetAnalogOutputArray_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_init)
  {
    (void)_init;
  }

  explicit SetAnalogOutputArray_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Response
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_io_msgs__srv__SetAnalogOutputArray_Response
    std::shared_ptr<robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetAnalogOutputArray_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetAnalogOutputArray_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetAnalogOutputArray_Response_

// alias to use template instance with default allocator
using SetAnalogOutputArray_Response =
  robotnik_io_msgs::srv::SetAnalogOutputArray_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotnik_io_msgs

namespace robotnik_io_msgs
{

namespace srv
{

struct SetAnalogOutputArray
{
  using Request = robotnik_io_msgs::srv::SetAnalogOutputArray_Request;
  using Response = robotnik_io_msgs::srv::SetAnalogOutputArray_Response;
};

}  // namespace srv

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_ANALOG_OUTPUT_ARRAY__STRUCT_HPP_
