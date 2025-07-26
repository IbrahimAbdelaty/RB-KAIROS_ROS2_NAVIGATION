// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robotnik_battery_msgs:msg/InverterStatusStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robotnik_battery_msgs/msg/detail/inverter_status_stamped__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robotnik_battery_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void InverterStatusStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robotnik_battery_msgs::msg::InverterStatusStamped(_init);
}

void InverterStatusStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robotnik_battery_msgs::msg::InverterStatusStamped *>(message_memory);
  typed_message->~InverterStatusStamped();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember InverterStatusStamped_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs::msg::InverterStatusStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_battery_msgs::msg::InverterStatus>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_battery_msgs::msg::InverterStatusStamped, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers InverterStatusStamped_message_members = {
  "robotnik_battery_msgs::msg",  // message namespace
  "InverterStatusStamped",  // message name
  2,  // number of fields
  sizeof(robotnik_battery_msgs::msg::InverterStatusStamped),
  InverterStatusStamped_message_member_array,  // message members
  InverterStatusStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  InverterStatusStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t InverterStatusStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &InverterStatusStamped_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robotnik_battery_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotnik_battery_msgs::msg::InverterStatusStamped>()
{
  return &::robotnik_battery_msgs::msg::rosidl_typesupport_introspection_cpp::InverterStatusStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_battery_msgs, msg, InverterStatusStamped)() {
  return &::robotnik_battery_msgs::msg::rosidl_typesupport_introspection_cpp::InverterStatusStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
