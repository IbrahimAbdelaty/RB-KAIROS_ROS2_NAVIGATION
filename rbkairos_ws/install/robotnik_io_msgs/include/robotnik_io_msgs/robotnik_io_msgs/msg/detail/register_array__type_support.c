// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotnik_io_msgs/msg/detail/register_array__rosidl_typesupport_introspection_c.h"
#include "robotnik_io_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotnik_io_msgs/msg/detail/register_array__functions.h"
#include "robotnik_io_msgs/msg/detail/register_array__struct.h"


// Include directives for member types
// Member `registers`
#include "robotnik_io_msgs/msg/register.h"
// Member `registers`
#include "robotnik_io_msgs/msg/detail/register__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_io_msgs__msg__RegisterArray__init(message_memory);
}

void robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_fini_function(void * message_memory)
{
  robotnik_io_msgs__msg__RegisterArray__fini(message_memory);
}

size_t robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__size_function__RegisterArray__registers(
  const void * untyped_member)
{
  const robotnik_io_msgs__msg__Register__Sequence * member =
    (const robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__get_const_function__RegisterArray__registers(
  const void * untyped_member, size_t index)
{
  const robotnik_io_msgs__msg__Register__Sequence * member =
    (const robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__get_function__RegisterArray__registers(
  void * untyped_member, size_t index)
{
  robotnik_io_msgs__msg__Register__Sequence * member =
    (robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__fetch_function__RegisterArray__registers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robotnik_io_msgs__msg__Register * item =
    ((const robotnik_io_msgs__msg__Register *)
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__get_const_function__RegisterArray__registers(untyped_member, index));
  robotnik_io_msgs__msg__Register * value =
    (robotnik_io_msgs__msg__Register *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__assign_function__RegisterArray__registers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robotnik_io_msgs__msg__Register * item =
    ((robotnik_io_msgs__msg__Register *)
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__get_function__RegisterArray__registers(untyped_member, index));
  const robotnik_io_msgs__msg__Register * value =
    (const robotnik_io_msgs__msg__Register *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__resize_function__RegisterArray__registers(
  void * untyped_member, size_t size)
{
  robotnik_io_msgs__msg__Register__Sequence * member =
    (robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  robotnik_io_msgs__msg__Register__Sequence__fini(member);
  return robotnik_io_msgs__msg__Register__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_member_array[1] = {
  {
    "registers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__msg__RegisterArray, registers),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__size_function__RegisterArray__registers,  // size() function pointer
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__get_const_function__RegisterArray__registers,  // get_const(index) function pointer
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__get_function__RegisterArray__registers,  // get(index) function pointer
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__fetch_function__RegisterArray__registers,  // fetch(index, &value) function pointer
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__assign_function__RegisterArray__registers,  // assign(index, value) function pointer
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__resize_function__RegisterArray__registers  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_members = {
  "robotnik_io_msgs__msg",  // message namespace
  "RegisterArray",  // message name
  1,  // number of fields
  sizeof(robotnik_io_msgs__msg__RegisterArray),
  robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_member_array,  // message members
  robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_type_support_handle = {
  0,
  &robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_io_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, RegisterArray)() {
  robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, Register)();
  if (!robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_type_support_handle.typesupport_identifier) {
    robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_io_msgs__msg__RegisterArray__rosidl_typesupport_introspection_c__RegisterArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
