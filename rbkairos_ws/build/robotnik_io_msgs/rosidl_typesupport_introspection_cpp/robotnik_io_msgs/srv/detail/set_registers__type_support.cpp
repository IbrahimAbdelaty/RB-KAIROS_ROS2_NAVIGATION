// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robotnik_io_msgs:srv/SetRegisters.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robotnik_io_msgs/srv/detail/set_registers__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robotnik_io_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetRegisters_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robotnik_io_msgs::srv::SetRegisters_Request(_init);
}

void SetRegisters_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robotnik_io_msgs::srv::SetRegisters_Request *>(message_memory);
  typed_message->~SetRegisters_Request();
}

size_t size_function__SetRegisters_Request__registers(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetRegisters_Request__registers(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  return &member[index];
}

void * get_function__SetRegisters_Request__registers(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetRegisters_Request__registers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const robotnik_io_msgs::msg::Register *>(
    get_const_function__SetRegisters_Request__registers(untyped_member, index));
  auto & value = *reinterpret_cast<robotnik_io_msgs::msg::Register *>(untyped_value);
  value = item;
}

void assign_function__SetRegisters_Request__registers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<robotnik_io_msgs::msg::Register *>(
    get_function__SetRegisters_Request__registers(untyped_member, index));
  const auto & value = *reinterpret_cast<const robotnik_io_msgs::msg::Register *>(untyped_value);
  item = value;
}

void resize_function__SetRegisters_Request__registers(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetRegisters_Request_message_member_array[1] = {
  {
    "registers",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_io_msgs::msg::Register>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::srv::SetRegisters_Request, registers),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetRegisters_Request__registers,  // size() function pointer
    get_const_function__SetRegisters_Request__registers,  // get_const(index) function pointer
    get_function__SetRegisters_Request__registers,  // get(index) function pointer
    fetch_function__SetRegisters_Request__registers,  // fetch(index, &value) function pointer
    assign_function__SetRegisters_Request__registers,  // assign(index, value) function pointer
    resize_function__SetRegisters_Request__registers  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetRegisters_Request_message_members = {
  "robotnik_io_msgs::srv",  // message namespace
  "SetRegisters_Request",  // message name
  1,  // number of fields
  sizeof(robotnik_io_msgs::srv::SetRegisters_Request),
  SetRegisters_Request_message_member_array,  // message members
  SetRegisters_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SetRegisters_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetRegisters_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetRegisters_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace robotnik_io_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotnik_io_msgs::srv::SetRegisters_Request>()
{
  return &::robotnik_io_msgs::srv::rosidl_typesupport_introspection_cpp::SetRegisters_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_io_msgs, srv, SetRegisters_Request)() {
  return &::robotnik_io_msgs::srv::rosidl_typesupport_introspection_cpp::SetRegisters_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/set_registers__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robotnik_io_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SetRegisters_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robotnik_io_msgs::srv::SetRegisters_Response(_init);
}

void SetRegisters_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robotnik_io_msgs::srv::SetRegisters_Response *>(message_memory);
  typed_message->~SetRegisters_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetRegisters_Response_message_member_array[1] = {
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_common_msgs::msg::Response>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::srv::SetRegisters_Response, response),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetRegisters_Response_message_members = {
  "robotnik_io_msgs::srv",  // message namespace
  "SetRegisters_Response",  // message name
  1,  // number of fields
  sizeof(robotnik_io_msgs::srv::SetRegisters_Response),
  SetRegisters_Response_message_member_array,  // message members
  SetRegisters_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SetRegisters_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetRegisters_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetRegisters_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace robotnik_io_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotnik_io_msgs::srv::SetRegisters_Response>()
{
  return &::robotnik_io_msgs::srv::rosidl_typesupport_introspection_cpp::SetRegisters_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_io_msgs, srv, SetRegisters_Response)() {
  return &::robotnik_io_msgs::srv::rosidl_typesupport_introspection_cpp::SetRegisters_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/set_registers__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace robotnik_io_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SetRegisters_service_members = {
  "robotnik_io_msgs::srv",  // service namespace
  "SetRegisters",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<robotnik_io_msgs::srv::SetRegisters>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SetRegisters_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetRegisters_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace robotnik_io_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robotnik_io_msgs::srv::SetRegisters>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::robotnik_io_msgs::srv::rosidl_typesupport_introspection_cpp::SetRegisters_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::robotnik_io_msgs::srv::SetRegisters_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::robotnik_io_msgs::srv::SetRegisters_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_io_msgs, srv, SetRegisters)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<robotnik_io_msgs::srv::SetRegisters>();
}

#ifdef __cplusplus
}
#endif
