// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotnik_supervisor_msgs:srv/GetModuleList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotnik_supervisor_msgs/srv/detail/get_module_list__rosidl_typesupport_introspection_c.h"
#include "robotnik_supervisor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotnik_supervisor_msgs/srv/detail/get_module_list__functions.h"
#include "robotnik_supervisor_msgs/srv/detail/get_module_list__struct.h"


// Include directives for member types
// Member `stack`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_supervisor_msgs__srv__GetModuleList_Request__init(message_memory);
}

void robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_fini_function(void * message_memory)
{
  robotnik_supervisor_msgs__srv__GetModuleList_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_member_array[1] = {
  {
    "stack",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_supervisor_msgs__srv__GetModuleList_Request, stack),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_members = {
  "robotnik_supervisor_msgs__srv",  // message namespace
  "GetModuleList_Request",  // message name
  1,  // number of fields
  sizeof(robotnik_supervisor_msgs__srv__GetModuleList_Request),
  robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_member_array,  // message members
  robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_type_support_handle = {
  0,
  &robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_supervisor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList_Request)() {
  if (!robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_type_support_handle.typesupport_identifier) {
    robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_supervisor_msgs__srv__GetModuleList_Request__rosidl_typesupport_introspection_c__GetModuleList_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_module_list__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robotnik_supervisor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_module_list__functions.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_module_list__struct.h"


// Include directives for member types
// Member `response`
#include "robotnik_common_msgs/msg/response.h"
// Member `response`
#include "robotnik_common_msgs/msg/detail/response__rosidl_typesupport_introspection_c.h"
// Member `modules`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotnik_supervisor_msgs__srv__GetModuleList_Response__init(message_memory);
}

void robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_fini_function(void * message_memory)
{
  robotnik_supervisor_msgs__srv__GetModuleList_Response__fini(message_memory);
}

size_t robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__size_function__GetModuleList_Response__modules(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__get_const_function__GetModuleList_Response__modules(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__get_function__GetModuleList_Response__modules(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__fetch_function__GetModuleList_Response__modules(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__get_const_function__GetModuleList_Response__modules(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__assign_function__GetModuleList_Response__modules(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__get_function__GetModuleList_Response__modules(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__resize_function__GetModuleList_Response__modules(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_member_array[2] = {
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_supervisor_msgs__srv__GetModuleList_Response, response),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "modules",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotnik_supervisor_msgs__srv__GetModuleList_Response, modules),  // bytes offset in struct
    NULL,  // default value
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__size_function__GetModuleList_Response__modules,  // size() function pointer
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__get_const_function__GetModuleList_Response__modules,  // get_const(index) function pointer
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__get_function__GetModuleList_Response__modules,  // get(index) function pointer
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__fetch_function__GetModuleList_Response__modules,  // fetch(index, &value) function pointer
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__assign_function__GetModuleList_Response__modules,  // assign(index, value) function pointer
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__resize_function__GetModuleList_Response__modules  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_members = {
  "robotnik_supervisor_msgs__srv",  // message namespace
  "GetModuleList_Response",  // message name
  2,  // number of fields
  sizeof(robotnik_supervisor_msgs__srv__GetModuleList_Response),
  robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_member_array,  // message members
  robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_type_support_handle = {
  0,
  &robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_supervisor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList_Response)() {
  robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_common_msgs, msg, Response)();
  if (!robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_type_support_handle.typesupport_identifier) {
    robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robotnik_supervisor_msgs__srv__GetModuleList_Response__rosidl_typesupport_introspection_c__GetModuleList_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robotnik_supervisor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_module_list__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_members = {
  "robotnik_supervisor_msgs__srv",  // service namespace
  "GetModuleList",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_Request_message_type_support_handle,
  NULL  // response message
  // robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_Response_message_type_support_handle
};

static rosidl_service_type_support_t robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_type_support_handle = {
  0,
  &robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotnik_supervisor_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList)() {
  if (!robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_type_support_handle.typesupport_identifier) {
    robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetModuleList_Response)()->data;
  }

  return &robotnik_supervisor_msgs__srv__detail__get_module_list__rosidl_typesupport_introspection_c__GetModuleList_service_type_support_handle;
}
