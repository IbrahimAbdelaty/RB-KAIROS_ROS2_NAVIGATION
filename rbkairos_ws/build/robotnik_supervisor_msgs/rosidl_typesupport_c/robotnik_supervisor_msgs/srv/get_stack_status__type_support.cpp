// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from robotnik_supervisor_msgs:srv/GetStackStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robotnik_supervisor_msgs/srv/detail/get_stack_status__struct.h"
#include "robotnik_supervisor_msgs/srv/detail/get_stack_status__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetStackStatus_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStackStatus_Request_type_support_ids_t;

static const _GetStackStatus_Request_type_support_ids_t _GetStackStatus_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetStackStatus_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStackStatus_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStackStatus_Request_type_support_symbol_names_t _GetStackStatus_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robotnik_supervisor_msgs, srv, GetStackStatus_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetStackStatus_Request)),
  }
};

typedef struct _GetStackStatus_Request_type_support_data_t
{
  void * data[2];
} _GetStackStatus_Request_type_support_data_t;

static _GetStackStatus_Request_type_support_data_t _GetStackStatus_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStackStatus_Request_message_typesupport_map = {
  2,
  "robotnik_supervisor_msgs",
  &_GetStackStatus_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetStackStatus_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetStackStatus_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetStackStatus_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStackStatus_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robotnik_supervisor_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robotnik_supervisor_msgs, srv, GetStackStatus_Request)() {
  return &::robotnik_supervisor_msgs::srv::rosidl_typesupport_c::GetStackStatus_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_stack_status__struct.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_stack_status__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetStackStatus_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStackStatus_Response_type_support_ids_t;

static const _GetStackStatus_Response_type_support_ids_t _GetStackStatus_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetStackStatus_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStackStatus_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStackStatus_Response_type_support_symbol_names_t _GetStackStatus_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robotnik_supervisor_msgs, srv, GetStackStatus_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetStackStatus_Response)),
  }
};

typedef struct _GetStackStatus_Response_type_support_data_t
{
  void * data[2];
} _GetStackStatus_Response_type_support_data_t;

static _GetStackStatus_Response_type_support_data_t _GetStackStatus_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStackStatus_Response_message_typesupport_map = {
  2,
  "robotnik_supervisor_msgs",
  &_GetStackStatus_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetStackStatus_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetStackStatus_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetStackStatus_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStackStatus_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robotnik_supervisor_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robotnik_supervisor_msgs, srv, GetStackStatus_Response)() {
  return &::robotnik_supervisor_msgs::srv::rosidl_typesupport_c::GetStackStatus_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/get_stack_status__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetStackStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetStackStatus_type_support_ids_t;

static const _GetStackStatus_type_support_ids_t _GetStackStatus_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetStackStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetStackStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetStackStatus_type_support_symbol_names_t _GetStackStatus_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robotnik_supervisor_msgs, srv, GetStackStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotnik_supervisor_msgs, srv, GetStackStatus)),
  }
};

typedef struct _GetStackStatus_type_support_data_t
{
  void * data[2];
} _GetStackStatus_type_support_data_t;

static _GetStackStatus_type_support_data_t _GetStackStatus_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetStackStatus_service_typesupport_map = {
  2,
  "robotnik_supervisor_msgs",
  &_GetStackStatus_service_typesupport_ids.typesupport_identifier[0],
  &_GetStackStatus_service_typesupport_symbol_names.symbol_name[0],
  &_GetStackStatus_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetStackStatus_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetStackStatus_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace robotnik_supervisor_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robotnik_supervisor_msgs, srv, GetStackStatus)() {
  return &::robotnik_supervisor_msgs::srv::rosidl_typesupport_c::GetStackStatus_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
