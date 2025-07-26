// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotnik_io_msgs:srv/GetRegisters.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotnik_io_msgs/srv/detail/get_registers__rosidl_typesupport_introspection_c.h"
#include "robotnik_io_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotnik_io_msgs/srv/detail/get_registers__functions.h"
#include "robotnik_io_msgs/srv/detail/get_registers__struct.h"


// Include directives for member types
// Member `addresses`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_io_msgs__srv__GetRegisters_Request__init(message_memory);
}

void robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_fini_function(void * message_memory)
{
  robotnik_io_msgs__srv__GetRegisters_Request__fini(message_memory);
}

size_t robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__size_function__GetRegisters_Request__addresses(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__get_const_function__GetRegisters_Request__addresses(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__get_function__GetRegisters_Request__addresses(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__fetch_function__GetRegisters_Request__addresses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__get_const_function__GetRegisters_Request__addresses(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__assign_function__GetRegisters_Request__addresses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__get_function__GetRegisters_Request__addresses(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__resize_function__GetRegisters_Request__addresses(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_member_array[1] = {
  {
    "addresses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__srv__GetRegisters_Request, addresses),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__size_function__GetRegisters_Request__addresses,  // size() function pointer
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__get_const_function__GetRegisters_Request__addresses,  // get_const(index) function pointer
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__get_function__GetRegisters_Request__addresses,  // get(index) function pointer
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__fetch_function__GetRegisters_Request__addresses,  // fetch(index, &value) function pointer
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__assign_function__GetRegisters_Request__addresses,  // assign(index, value) function pointer
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__resize_function__GetRegisters_Request__addresses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_members = {
  "robotnik_io_msgs__srv",  // message namespace
  "GetRegisters_Request",  // message name
  1,  // number of fields
  sizeof(robotnik_io_msgs__srv__GetRegisters_Request),
  robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_member_array,  // message members
  robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_type_support_handle = {
  0,
  &robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_io_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters_Request)() {
  if (!robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_type_support_handle.typesupport_identifier) {
    robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_io_msgs__srv__GetRegisters_Request__rosidl_typesupport_introspection_c__GetRegisters_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robotnik_io_msgs/srv/detail/get_registers__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robotnik_io_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/get_registers__functions.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/get_registers__struct.h"


// Include directives for member types
// Member `response`
#include "robotnik_common_msgs/msg/response.h"
// Member `response`
#include "robotnik_common_msgs/msg/detail/response__rosidl_typesupport_introspection_c.h"
// Member `registers`
#include "robotnik_io_msgs/msg/register.h"
// Member `registers`
#include "robotnik_io_msgs/msg/detail/register__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_io_msgs__srv__GetRegisters_Response__init(message_memory);
}

void robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_fini_function(void * message_memory)
{
  robotnik_io_msgs__srv__GetRegisters_Response__fini(message_memory);
}

size_t robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__size_function__GetRegisters_Response__registers(
  const void * untyped_member)
{
  const robotnik_io_msgs__msg__Register__Sequence * member =
    (const robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__get_const_function__GetRegisters_Response__registers(
  const void * untyped_member, size_t index)
{
  const robotnik_io_msgs__msg__Register__Sequence * member =
    (const robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__get_function__GetRegisters_Response__registers(
  void * untyped_member, size_t index)
{
  robotnik_io_msgs__msg__Register__Sequence * member =
    (robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__fetch_function__GetRegisters_Response__registers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const robotnik_io_msgs__msg__Register * item =
    ((const robotnik_io_msgs__msg__Register *)
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__get_const_function__GetRegisters_Response__registers(untyped_member, index));
  robotnik_io_msgs__msg__Register * value =
    (robotnik_io_msgs__msg__Register *)(untyped_value);
  *value = *item;
}

void robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__assign_function__GetRegisters_Response__registers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  robotnik_io_msgs__msg__Register * item =
    ((robotnik_io_msgs__msg__Register *)
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__get_function__GetRegisters_Response__registers(untyped_member, index));
  const robotnik_io_msgs__msg__Register * value =
    (const robotnik_io_msgs__msg__Register *)(untyped_value);
  *item = *value;
}

bool robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__resize_function__GetRegisters_Response__registers(
  void * untyped_member, size_t size)
{
  robotnik_io_msgs__msg__Register__Sequence * member =
    (robotnik_io_msgs__msg__Register__Sequence *)(untyped_member);
  robotnik_io_msgs__msg__Register__Sequence__fini(member);
  return robotnik_io_msgs__msg__Register__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_member_array[2] = {
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__srv__GetRegisters_Response, response),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "registers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_io_msgs__srv__GetRegisters_Response, registers),  // bytes offset in struct
    NULL,  // default value
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__size_function__GetRegisters_Response__registers,  // size() function pointer
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__get_const_function__GetRegisters_Response__registers,  // get_const(index) function pointer
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__get_function__GetRegisters_Response__registers,  // get(index) function pointer
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__fetch_function__GetRegisters_Response__registers,  // fetch(index, &value) function pointer
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__assign_function__GetRegisters_Response__registers,  // assign(index, value) function pointer
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__resize_function__GetRegisters_Response__registers  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_members = {
  "robotnik_io_msgs__srv",  // message namespace
  "GetRegisters_Response",  // message name
  2,  // number of fields
  sizeof(robotnik_io_msgs__srv__GetRegisters_Response),
  robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_member_array,  // message members
  robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_type_support_handle = {
  0,
  &robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_io_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters_Response)() {
  robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_common_msgs, msg, Response)();
  robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, msg, Register)();
  if (!robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_type_support_handle.typesupport_identifier) {
    robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_io_msgs__srv__GetRegisters_Response__rosidl_typesupport_introspection_c__GetRegisters_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robotnik_io_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/get_registers__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_members = {
  "robotnik_io_msgs__srv",  // service namespace
  "GetRegisters",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_Request_message_type_support_handle,
  NULL  // response message
  // robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_Response_message_type_support_handle
};

static rosidl_service_type_support_t robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_type_support_handle = {
  0,
  &robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_io_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters)() {
  if (!robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_type_support_handle.typesupport_identifier) {
    robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_io_msgs, srv, GetRegisters_Response)()->data;
  }

  return &robotnik_io_msgs__srv__detail__get_registers__rosidl_typesupport_introspection_c__GetRegisters_service_type_support_handle;
}
