// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_io_msgs:srv/SetDigitalOutputArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_DIGITAL_OUTPUT_ARRAY__STRUCT_H_
#define ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_DIGITAL_OUTPUT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'output'
#include "robotnik_io_msgs/msg/detail/digital_io__struct.h"

/// Struct defined in srv/SetDigitalOutputArray in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__srv__SetDigitalOutputArray_Request
{
  robotnik_io_msgs__msg__DigitalIO__Sequence output;
} robotnik_io_msgs__srv__SetDigitalOutputArray_Request;

// Struct for a sequence of robotnik_io_msgs__srv__SetDigitalOutputArray_Request.
typedef struct robotnik_io_msgs__srv__SetDigitalOutputArray_Request__Sequence
{
  robotnik_io_msgs__srv__SetDigitalOutputArray_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__srv__SetDigitalOutputArray_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in srv/SetDigitalOutputArray in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__srv__SetDigitalOutputArray_Response
{
  robotnik_common_msgs__msg__Response response;
} robotnik_io_msgs__srv__SetDigitalOutputArray_Response;

// Struct for a sequence of robotnik_io_msgs__srv__SetDigitalOutputArray_Response.
typedef struct robotnik_io_msgs__srv__SetDigitalOutputArray_Response__Sequence
{
  robotnik_io_msgs__srv__SetDigitalOutputArray_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__srv__SetDigitalOutputArray_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__SRV__DETAIL__SET_DIGITAL_OUTPUT_ARRAY__STRUCT_H_
