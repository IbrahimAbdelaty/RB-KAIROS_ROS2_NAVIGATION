// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_io_msgs:msg/InputsOutputs.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__STRUCT_H_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'digital_inputs'
// Member 'digital_outputs'
#include "robotnik_io_msgs/msg/detail/digital_io__struct.h"
// Member 'analog_inputs'
// Member 'analog_outputs'
#include "robotnik_io_msgs/msg/detail/analog_io__struct.h"

/// Struct defined in msg/InputsOutputs in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__msg__InputsOutputs
{
  robotnik_io_msgs__msg__DigitalIO__Sequence digital_inputs;
  robotnik_io_msgs__msg__DigitalIO__Sequence digital_outputs;
  robotnik_io_msgs__msg__AnalogIO__Sequence analog_inputs;
  robotnik_io_msgs__msg__AnalogIO__Sequence analog_outputs;
} robotnik_io_msgs__msg__InputsOutputs;

// Struct for a sequence of robotnik_io_msgs__msg__InputsOutputs.
typedef struct robotnik_io_msgs__msg__InputsOutputs__Sequence
{
  robotnik_io_msgs__msg__InputsOutputs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__msg__InputsOutputs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__STRUCT_H_
