// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotnik_battery_msgs:msg/BatteryStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__FUNCTIONS_H_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotnik_battery_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotnik_battery_msgs/msg/detail/battery_status_stamped__struct.h"

/// Initialize msg/BatteryStatusStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_battery_msgs__msg__BatteryStatusStamped
 * )) before or use
 * robotnik_battery_msgs__msg__BatteryStatusStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__BatteryStatusStamped__init(robotnik_battery_msgs__msg__BatteryStatusStamped * msg);

/// Finalize msg/BatteryStatusStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__BatteryStatusStamped__fini(robotnik_battery_msgs__msg__BatteryStatusStamped * msg);

/// Create msg/BatteryStatusStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_battery_msgs__msg__BatteryStatusStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
robotnik_battery_msgs__msg__BatteryStatusStamped *
robotnik_battery_msgs__msg__BatteryStatusStamped__create();

/// Destroy msg/BatteryStatusStamped message.
/**
 * It calls
 * robotnik_battery_msgs__msg__BatteryStatusStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__BatteryStatusStamped__destroy(robotnik_battery_msgs__msg__BatteryStatusStamped * msg);

/// Check for msg/BatteryStatusStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__BatteryStatusStamped__are_equal(const robotnik_battery_msgs__msg__BatteryStatusStamped * lhs, const robotnik_battery_msgs__msg__BatteryStatusStamped * rhs);

/// Copy a msg/BatteryStatusStamped message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__BatteryStatusStamped__copy(
  const robotnik_battery_msgs__msg__BatteryStatusStamped * input,
  robotnik_battery_msgs__msg__BatteryStatusStamped * output);

/// Initialize array of msg/BatteryStatusStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_battery_msgs__msg__BatteryStatusStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__init(robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * array, size_t size);

/// Finalize array of msg/BatteryStatusStamped messages.
/**
 * It calls
 * robotnik_battery_msgs__msg__BatteryStatusStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__fini(robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * array);

/// Create array of msg/BatteryStatusStamped messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence *
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__create(size_t size);

/// Destroy array of msg/BatteryStatusStamped messages.
/**
 * It calls
 * robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__destroy(robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * array);

/// Check for msg/BatteryStatusStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__are_equal(const robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * lhs, const robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * rhs);

/// Copy an array of msg/BatteryStatusStamped messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence__copy(
  const robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * input,
  robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__FUNCTIONS_H_
