// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robotnik_controllers_msgs:srv/AvailableKinematics.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__STRUCT_HPP_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Request __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Request __declspec(deprecated)
#endif

namespace robotnik_controllers_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AvailableKinematics_Request_
{
  using Type = AvailableKinematics_Request_<ContainerAllocator>;

  explicit AvailableKinematics_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit AvailableKinematics_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Request
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Request
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AvailableKinematics_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const AvailableKinematics_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AvailableKinematics_Request_

// alias to use template instance with default allocator
using AvailableKinematics_Request =
  robotnik_controllers_msgs::srv::AvailableKinematics_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotnik_controllers_msgs


// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Response __attribute__((deprecated))
#else
# define DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Response __declspec(deprecated)
#endif

namespace robotnik_controllers_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AvailableKinematics_Response_
{
  using Type = AvailableKinematics_Response_<ContainerAllocator>;

  explicit AvailableKinematics_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_init)
  {
    (void)_init;
  }

  explicit AvailableKinematics_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _response_type =
    robotnik_common_msgs::msg::Response_<ContainerAllocator>;
  _response_type response;
  using _kinematics_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _kinematics_type kinematics;

  // setters for named parameter idiom
  Type & set__response(
    const robotnik_common_msgs::msg::Response_<ContainerAllocator> & _arg)
  {
    this->response = _arg;
    return *this;
  }
  Type & set__kinematics(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->kinematics = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Response
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robotnik_controllers_msgs__srv__AvailableKinematics_Response
    std::shared_ptr<robotnik_controllers_msgs::srv::AvailableKinematics_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AvailableKinematics_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    if (this->kinematics != other.kinematics) {
      return false;
    }
    return true;
  }
  bool operator!=(const AvailableKinematics_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AvailableKinematics_Response_

// alias to use template instance with default allocator
using AvailableKinematics_Response =
  robotnik_controllers_msgs::srv::AvailableKinematics_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robotnik_controllers_msgs

namespace robotnik_controllers_msgs
{

namespace srv
{

struct AvailableKinematics
{
  using Request = robotnik_controllers_msgs::srv::AvailableKinematics_Request;
  using Response = robotnik_controllers_msgs::srv::AvailableKinematics_Response;
};

}  // namespace srv

}  // namespace robotnik_controllers_msgs

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__STRUCT_HPP_
