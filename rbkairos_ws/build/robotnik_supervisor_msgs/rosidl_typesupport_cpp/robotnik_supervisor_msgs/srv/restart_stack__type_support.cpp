// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from robotnik_supervisor_msgs:srv/RestartStack.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robotnik_supervisor_msgs/srv/detail/restart_stack__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RestartStack_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RestartStack_Request_type_support_ids_t;

static const _RestartStack_Request_type_support_ids_t _RestartStack_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RestartStack_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RestartStack_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RestartStack_Request_type_support_symbol_names_t _RestartStack_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robotnik_supervisor_msgs, srv, RestartStack_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_supervisor_msgs, srv, RestartStack_Request)),
  }
};

typedef struct _RestartStack_Request_type_support_data_t
{
  void * data[2];
} _RestartStack_Request_type_support_data_t;

static _RestartStack_Request_type_support_data_t _RestartStack_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RestartStack_Request_message_typesupport_map = {
  2,
  "robotnik_supervisor_msgs",
  &_RestartStack_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RestartStack_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RestartStack_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RestartStack_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RestartStack_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robotnik_supervisor_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotnik_supervisor_msgs::srv::RestartStack_Request>()
{
  return &::robotnik_supervisor_msgs::srv::rosidl_typesupport_cpp::RestartStack_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robotnik_supervisor_msgs, srv, RestartStack_Request)() {
  return get_message_type_support_handle<robotnik_supervisor_msgs::srv::RestartStack_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/restart_stack__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RestartStack_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RestartStack_Response_type_support_ids_t;

static const _RestartStack_Response_type_support_ids_t _RestartStack_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RestartStack_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RestartStack_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RestartStack_Response_type_support_symbol_names_t _RestartStack_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robotnik_supervisor_msgs, srv, RestartStack_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_supervisor_msgs, srv, RestartStack_Response)),
  }
};

typedef struct _RestartStack_Response_type_support_data_t
{
  void * data[2];
} _RestartStack_Response_type_support_data_t;

static _RestartStack_Response_type_support_data_t _RestartStack_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RestartStack_Response_message_typesupport_map = {
  2,
  "robotnik_supervisor_msgs",
  &_RestartStack_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RestartStack_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RestartStack_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RestartStack_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RestartStack_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robotnik_supervisor_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robotnik_supervisor_msgs::srv::RestartStack_Response>()
{
  return &::robotnik_supervisor_msgs::srv::rosidl_typesupport_cpp::RestartStack_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robotnik_supervisor_msgs, srv, RestartStack_Response)() {
  return get_message_type_support_handle<robotnik_supervisor_msgs::srv::RestartStack_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robotnik_supervisor_msgs/srv/detail/restart_stack__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robotnik_supervisor_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RestartStack_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RestartStack_type_support_ids_t;

static const _RestartStack_type_support_ids_t _RestartStack_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RestartStack_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RestartStack_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RestartStack_type_support_symbol_names_t _RestartStack_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robotnik_supervisor_msgs, srv, RestartStack)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robotnik_supervisor_msgs, srv, RestartStack)),
  }
};

typedef struct _RestartStack_type_support_data_t
{
  void * data[2];
} _RestartStack_type_support_data_t;

static _RestartStack_type_support_data_t _RestartStack_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RestartStack_service_typesupport_map = {
  2,
  "robotnik_supervisor_msgs",
  &_RestartStack_service_typesupport_ids.typesupport_identifier[0],
  &_RestartStack_service_typesupport_symbol_names.symbol_name[0],
  &_RestartStack_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RestartStack_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RestartStack_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace robotnik_supervisor_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robotnik_supervisor_msgs::srv::RestartStack>()
{
  return &::robotnik_supervisor_msgs::srv::rosidl_typesupport_cpp::RestartStack_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robotnik_supervisor_msgs, srv, RestartStack)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robotnik_supervisor_msgs::srv::RestartStack>();
}

#ifdef __cplusplus
}
#endif
