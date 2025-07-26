// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_io_msgs:msg/Register.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__STRUCT_H_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Register in the package robotnik_io_msgs.
typedef struct robotnik_io_msgs__msg__Register
{
  int32_t address;
  int32_t value;
} robotnik_io_msgs__msg__Register;

// Struct for a sequence of robotnik_io_msgs__msg__Register.
typedef struct robotnik_io_msgs__msg__Register__Sequence
{
  robotnik_io_msgs__msg__Register * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_io_msgs__msg__Register__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER__STRUCT_H_
