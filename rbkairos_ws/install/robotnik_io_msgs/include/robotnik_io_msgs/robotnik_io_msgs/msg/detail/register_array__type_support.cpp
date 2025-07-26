// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robotnik_io_msgs/msg/detail/register_array__struct.hpp"
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

void RegisterArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robotnik_io_msgs::msg::RegisterArray(_init);
}

void RegisterArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robotnik_io_msgs::msg::RegisterArray *>(message_memory);
  typed_message->~RegisterArray();
}

size_t size_function__RegisterArray__registers(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RegisterArray__registers(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  return &member[index];
}

void * get_function__RegisterArray__registers(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  return &member[index];
}

void fetch_function__RegisterArray__registers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const robotnik_io_msgs::msg::Register *>(
    get_const_function__RegisterArray__registers(untyped_member, index));
  auto & value = *reinterpret_cast<robotnik_io_msgs::msg::Register *>(untyped_value);
  value = item;
}

void assign_function__RegisterArray__registers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<robotnik_io_msgs::msg::Register *>(
    get_function__RegisterArray__registers(untyped_member, index));
  const auto & value = *reinterpret_cast<const robotnik_io_msgs::msg::Register *>(untyped_value);
  item = value;
}

void resize_function__RegisterArray__registers(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<robotnik_io_msgs::msg::Register> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RegisterArray_message_member_array[1] = {
  {
    "registers",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<robotnik_io_msgs::msg::Register>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs::msg::RegisterArray, registers),  // bytes offset in struct
    nullptr,  // default value
    size_function__RegisterArray__registers,  // size() function pointer
    get_const_function__RegisterArray__registers,  // get_const(index) function pointer
    get_function__RegisterArray__registers,  // get(index) function pointer
    fetch_function__RegisterArray__registers,  // fetch(index, &value) function pointer
    assign_function__RegisterArray__registers,  // assign(index, value) function pointer
    resize_function__RegisterArray__registers  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RegisterArray_message_members = {
  "robotnik_io_msgs::msg",  // message namespace
  "RegisterArray",  // message name
  1,  // number of fields
  sizeof(robotnik_io_msgs::msg::RegisterArray),
  RegisterArray_message_member_array,  // message members
  RegisterArray_init_function,  // function to initialize message memory (memory has to be allocated)
  RegisterArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RegisterArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RegisterArray_message_members,
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
get_message_type_support_handle<robotnik_io_msgs::msg::RegisterArray>()
{
  return &::robotnik_io_msgs::msg::rosidl_typesupport_introspection_cpp::RegisterArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_io_msgs, msg, RegisterArray)() {
  return &::robotnik_io_msgs::msg::rosidl_typesupport_introspection_cpp::RegisterArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
