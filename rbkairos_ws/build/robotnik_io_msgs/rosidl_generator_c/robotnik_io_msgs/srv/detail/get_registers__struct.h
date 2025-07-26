// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_io_msgs:srv/GetRegisters.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__STRUCT_H_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'addresses'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetRegisters in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__srv__GetRegisters_Request
{
  rosidl_runtime_c__int32__Sequence addresses;
} robotnik_io_msgs__srv__GetRegisters_Request;

// Struct for a sequence of robotnik_io_msgs__srv__GetRegisters_Request.
typedef struct robotnik_io_msgs__srv__GetRegisters_Request__Sequence
{
  robotnik_io_msgs__srv__GetRegisters_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__srv__GetRegisters_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"
// Member 'registers'
#include "robotnik_io_msgs/msg/detail/register__struct.h"

/// Struct defined in srv/GetRegisters in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__srv__GetRegisters_Response
{
  robotnik_common_msgs__msg__Response response;
  robotnik_io_msgs__msg__Register__Sequence registers;
} robotnik_io_msgs__srv__GetRegisters_Response;

// Struct for a sequence of robotnik_io_msgs__srv__GetRegisters_Response.
typedef struct robotnik_io_msgs__srv__GetRegisters_Response__Sequence
{
  robotnik_io_msgs__srv__GetRegisters_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__srv__GetRegisters_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__GET_REGISTERS__STRUCT_H_
