// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robotnik_io_msgs:msg/InputsOutputs.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robotnik_io_msgs/msg/detail/inputs_outputs__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robotnik_io_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void InputsOutputs_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robotnik_io_msgs::msg::InputsOutputs(_init);
}

void InputsOutputs_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robotnik_io_msgs::msg::InputsOutputs *>(message_memory);
  typed_message->~InputsOutputs();
}

size_t size_function__InputsOutputs__digital_inputs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  return member->size();
}

const void * get_const_function__InputsOutputs__digital_inputs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  return &member[index];
}

void * get_function__InputsOutputs__digital_inputs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  return &member[index];
}

void fetch_function__InputsOutputs__digital_inputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const robotnik_io_msgs::msg::DigitalIO *>(
    get_const_function__InputsOutputs__digital_inputs(untyped_member, index));
  auto & value = *reinterpret_cast<robotnik_io_msgs::msg::DigitalIO *>(untyped_value);
  value = item;
}

void assign_function__InputsOutputs__digital_inputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<robotnik_io_msgs::msg::DigitalIO *>(
    get_function__InputsOutputs__digital_inputs(untyped_member, index));
  const auto & value = *reinterpret_cast<const robotnik_io_msgs::msg::DigitalIO *>(untyped_value);
  item = value;
}

void resize_function__InputsOutputs__digital_inputs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  member->resize(size);
}

size_t size_function__InputsOutputs__digital_outputs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  return member->size();
}

const void * get_const_function__InputsOutputs__digital_outputs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  return &member[index];
}

void * get_function__InputsOutputs__digital_outputs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  return &member[index];
}

void fetch_function__InputsOutputs__digital_outputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const robotnik_io_msgs::msg::DigitalIO *>(
    get_const_function__InputsOutputs__digital_outputs(untyped_member, index));
  auto & value = *reinterpret_cast<robotnik_io_msgs::msg::DigitalIO *>(untyped_value);
  value = item;
}

void assign_function__InputsOutputs__digital_outputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<robotnik_io_msgs::msg::DigitalIO *>(
    get_function__InputsOutputs__digital_outputs(untyped_member, index));
  const auto & value = *reinterpret_cast<const robotnik_io_msgs::msg::DigitalIO *>(untyped_value);
  item = value;
}

void resize_function__InputsOutputs__digital_outputs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robotnik_io_msgs::msg::DigitalIO> *>(untyped_member);
  member->resize(size);
}

size_t size_function__InputsOutputs__analog_inputs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  return member->size();
}

const void * get_const_function__InputsOutputs__analog_inputs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  return &member[index];
}

void * get_function__InputsOutputs__analog_inputs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  return &member[index];
}

void fetch_function__InputsOutputs__analog_inputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const robotnik_io_msgs::msg::AnalogIO *>(
    get_const_function__InputsOutputs__analog_inputs(untyped_member, index));
  auto & value = *reinterpret_cast<robotnik_io_msgs::msg::AnalogIO *>(untyped_value);
  value = item;
}

void assign_function__InputsOutputs__analog_inputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<robotnik_io_msgs::msg::AnalogIO *>(
    get_function__InputsOutputs__analog_inputs(untyped_member, index));
  const auto & value = *reinterpret_cast<const robotnik_io_msgs::msg::AnalogIO *>(untyped_value);
  item = value;
}

void resize_function__InputsOutputs__analog_inputs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  member->resize(size);
}

size_t size_function__InputsOutputs__analog_outputs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  return member->size();
}

const void * get_const_function__InputsOutputs__analog_outputs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  return &member[index];
}

void * get_function__InputsOutputs__analog_outputs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  return &member[index];
}

void fetch_function__InputsOutputs__analog_outputs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const robotnik_io_msgs::msg::AnalogIO *>(
    get_const_function__InputsOutputs__analog_outputs(untyped_member, index));
  auto & value = *reinterpret_cast<robotnik_io_msgs::msg::AnalogIO *>(untyped_value);
  value = item;
}

void assign_function__InputsOutputs__analog_outputs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<robotnik_io_msgs::msg::AnalogIO *>(
    get_function__InputsOutputs__analog_outputs(untyped_member, index));
  const auto & value = *reinterpret_cast<const robotnik_io_msgs::msg::AnalogIO *>(untyped_value);
  item = value;
}

void resize_function__InputsOutputs__analog_outputs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robotnik_io_msgs::msg::AnalogIO> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember InputsOutputs_message_member_array[4] = {
  {
    "digital_inputs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_io_msgs::msg::DigitalIO>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::msg::InputsOutputs, digital_inputs),  // bytes offset in struct
    nullptr,  // default value
    size_function__InputsOutputs__digital_inputs,  // size() function pointer
    get_const_function__InputsOutputs__digital_inputs,  // get_const(index) function pointer
    get_function__InputsOutputs__digital_inputs,  // get(index) function pointer
    fetch_function__InputsOutputs__digital_inputs,  // fetch(index, &value) function pointer
    assign_function__InputsOutputs__digital_inputs,  // assign(index, value) function pointer
    resize_function__InputsOutputs__digital_inputs  // resize(index) function pointer
  },
  {
    "digital_outputs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_io_msgs::msg::DigitalIO>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::msg::InputsOutputs, digital_outputs),  // bytes offset in struct
    nullptr,  // default value
    size_function__InputsOutputs__digital_outputs,  // size() function pointer
    get_const_function__InputsOutputs__digital_outputs,  // get_const(index) function pointer
    get_function__InputsOutputs__digital_outputs,  // get(index) function pointer
    fetch_function__InputsOutputs__digital_outputs,  // fetch(index, &value) function pointer
    assign_function__InputsOutputs__digital_outputs,  // assign(index, value) function pointer
    resize_function__InputsOutputs__digital_outputs  // resize(index) function pointer
  },
  {
    "analog_inputs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_io_msgs::msg::AnalogIO>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::msg::InputsOutputs, analog_inputs),  // bytes offset in struct
    nullptr,  // default value
    size_function__InputsOutputs__analog_inputs,  // size() function pointer
    get_const_function__InputsOutputs__analog_inputs,  // get_const(index) function pointer
    get_function__InputsOutputs__analog_inputs,  // get(index) function pointer
    fetch_function__InputsOutputs__analog_inputs,  // fetch(index, &value) function pointer
    assign_function__InputsOutputs__analog_inputs,  // assign(index, value) function pointer
    resize_function__InputsOutputs__analog_inputs  // resize(index) function pointer
  },
  {
    "analog_outputs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_io_msgs::msg::AnalogIO>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::msg::InputsOutputs, analog_outputs),  // bytes offset in struct
    nullptr,  // default value
    size_function__InputsOutputs__analog_outputs,  // size() function pointer
    get_const_function__InputsOutputs__analog_outputs,  // get_const(index) function pointer
    get_function__InputsOutputs__analog_outputs,  // get(index) function pointer
    fetch_function__InputsOutputs__analog_outputs,  // fetch(index, &value) function pointer
    assign_function__InputsOutputs__analog_outputs,  // assign(index, value) function pointer
    resize_function__InputsOutputs__analog_outputs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers InputsOutputs_message_members = {
  "robotnik_io_msgs::msg",  // message namespace
  "InputsOutputs",  // message name
  4,  // number of fields
  sizeof(robotnik_io_msgs::msg::InputsOutputs),
  InputsOutputs_message_member_array,  // message members
  InputsOutputs_init_function,  // function to initialize message memory (memory has to be allocated)
  InputsOutputs_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t InputsOutputs_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &InputsOutputs_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robotnik_io_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotnik_io_msgs::msg::InputsOutputs>()
{
  return &::robotnik_io_msgs::msg::rosidl_typesupport_introspection_cpp::InputsOutputs_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_io_msgs, msg, InputsOutputs)() {
  return &::robotnik_io_msgs::msg::rosidl_typesupport_introspection_cpp::InputsOutputs_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
