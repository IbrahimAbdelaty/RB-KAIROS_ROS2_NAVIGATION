// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__FUNCTIONS_H_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotnik_io_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotnik_io_msgs/msg/detail/register_array__struct.h"

/// Initialize msg/RegisterArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_io_msgs__msg__RegisterArray
 * )) before or use
 * robotnik_io_msgs__msg__RegisterArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
bool
robotnik_io_msgs__msg__RegisterArray__init(robotnik_io_msgs__msg__RegisterArray * msg);

/// Finalize msg/RegisterArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
void
robotnik_io_msgs__msg__RegisterArray__fini(robotnik_io_msgs__msg__RegisterArray * msg);

/// Create msg/RegisterArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_io_msgs__msg__RegisterArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
robotnik_io_msgs__msg__RegisterArray *
robotnik_io_msgs__msg__RegisterArray__create();

/// Destroy msg/RegisterArray message.
/**
 * It calls
 * robotnik_io_msgs__msg__RegisterArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
void
robotnik_io_msgs__msg__RegisterArray__destroy(robotnik_io_msgs__msg__RegisterArray * msg);

/// Check for msg/RegisterArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
bool
robotnik_io_msgs__msg__RegisterArray__are_equal(const robotnik_io_msgs__msg__RegisterArray * lhs, const robotnik_io_msgs__msg__RegisterArray * rhs);

/// Copy a msg/RegisterArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
bool
robotnik_io_msgs__msg__RegisterArray__copy(
  const robotnik_io_msgs__msg__RegisterArray * input,
  robotnik_io_msgs__msg__RegisterArray * output);

/// Initialize array of msg/RegisterArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_io_msgs__msg__RegisterArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
bool
robotnik_io_msgs__msg__RegisterArray__Sequence__init(robotnik_io_msgs__msg__RegisterArray__Sequence * array, size_t size);

/// Finalize array of msg/RegisterArray messages.
/**
 * It calls
 * robotnik_io_msgs__msg__RegisterArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
void
robotnik_io_msgs__msg__RegisterArray__Sequence__fini(robotnik_io_msgs__msg__RegisterArray__Sequence * array);

/// Create array of msg/RegisterArray messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_io_msgs__msg__RegisterArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
robotnik_io_msgs__msg__RegisterArray__Sequence *
robotnik_io_msgs__msg__RegisterArray__Sequence__create(size_t size);

/// Destroy array of msg/RegisterArray messages.
/**
 * It calls
 * robotnik_io_msgs__msg__RegisterArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
void
robotnik_io_msgs__msg__RegisterArray__Sequence__destroy(robotnik_io_msgs__msg__RegisterArray__Sequence * array);

/// Check for msg/RegisterArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
bool
robotnik_io_msgs__msg__RegisterArray__Sequence__are_equal(const robotnik_io_msgs__msg__RegisterArray__Sequence * lhs, const robotnik_io_msgs__msg__RegisterArray__Sequence * rhs);

/// Copy an array of msg/RegisterArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_io_msgs
bool
robotnik_io_msgs__msg__RegisterArray__Sequence__copy(
  const robotnik_io_msgs__msg__RegisterArray__Sequence * input,
  robotnik_io_msgs__msg__RegisterArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__REGISTER_ARRAY__FUNCTIONS_H_
