// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotnik_battery_msgs:msg/DockingStationStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS_STAMPED__FUNCTIONS_H_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotnik_battery_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotnik_battery_msgs/msg/detail/docking_station_status_stamped__struct.h"

/// Initialize msg/DockingStationStatusStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_battery_msgs__msg__DockingStationStatusStamped
 * )) before or use
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__DockingStationStatusStamped__init(robotnik_battery_msgs__msg__DockingStationStatusStamped * msg);

/// Finalize msg/DockingStationStatusStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__DockingStationStatusStamped__fini(robotnik_battery_msgs__msg__DockingStationStatusStamped * msg);

/// Create msg/DockingStationStatusStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
robotnik_battery_msgs__msg__DockingStationStatusStamped *
robotnik_battery_msgs__msg__DockingStationStatusStamped__create();

/// Destroy msg/DockingStationStatusStamped message.
/**
 * It calls
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__DockingStationStatusStamped__destroy(robotnik_battery_msgs__msg__DockingStationStatusStamped * msg);

/// Check for msg/DockingStationStatusStamped message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__DockingStationStatusStamped__are_equal(const robotnik_battery_msgs__msg__DockingStationStatusStamped * lhs, const robotnik_battery_msgs__msg__DockingStationStatusStamped * rhs);

/// Copy a msg/DockingStationStatusStamped message.
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
robotnik_battery_msgs__msg__DockingStationStatusStamped__copy(
  const robotnik_battery_msgs__msg__DockingStationStatusStamped * input,
  robotnik_battery_msgs__msg__DockingStationStatusStamped * output);

/// Initialize array of msg/DockingStationStatusStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__init(robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * array, size_t size);

/// Finalize array of msg/DockingStationStatusStamped messages.
/**
 * It calls
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__fini(robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * array);

/// Create array of msg/DockingStationStatusStamped messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence *
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__create(size_t size);

/// Destroy array of msg/DockingStationStatusStamped messages.
/**
 * It calls
 * robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
void
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__destroy(robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * array);

/// Check for msg/DockingStationStatusStamped message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_battery_msgs
bool
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__are_equal(const robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * lhs, const robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * rhs);

/// Copy an array of msg/DockingStationStatusStamped messages.
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
robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence__copy(
  const robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * input,
  robotnik_battery_msgs__msg__DockingStationStatusStamped__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS_STAMPED__FUNCTIONS_H_
