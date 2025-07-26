// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotnik_controllers_msgs:srv/AvailableKinematics.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__FUNCTIONS_H_
#define ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotnik_controllers_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotnik_controllers_msgs/srv/detail/available_kinematics__struct.h"

/// Initialize srv/AvailableKinematics message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request
 * )) before or use
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Request__init(robotnik_controllers_msgs__srv__AvailableKinematics_Request * msg);

/// Finalize srv/AvailableKinematics message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Request__fini(robotnik_controllers_msgs__srv__AvailableKinematics_Request * msg);

/// Create srv/AvailableKinematics message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
robotnik_controllers_msgs__srv__AvailableKinematics_Request *
robotnik_controllers_msgs__srv__AvailableKinematics_Request__create();

/// Destroy srv/AvailableKinematics message.
/**
 * It calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Request__destroy(robotnik_controllers_msgs__srv__AvailableKinematics_Request * msg);

/// Check for srv/AvailableKinematics message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Request__are_equal(const robotnik_controllers_msgs__srv__AvailableKinematics_Request * lhs, const robotnik_controllers_msgs__srv__AvailableKinematics_Request * rhs);

/// Copy a srv/AvailableKinematics message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Request__copy(
  const robotnik_controllers_msgs__srv__AvailableKinematics_Request * input,
  robotnik_controllers_msgs__srv__AvailableKinematics_Request * output);

/// Initialize array of srv/AvailableKinematics messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__init(robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * array, size_t size);

/// Finalize array of srv/AvailableKinematics messages.
/**
 * It calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__fini(robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * array);

/// Create array of srv/AvailableKinematics messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence *
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__create(size_t size);

/// Destroy array of srv/AvailableKinematics messages.
/**
 * It calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__destroy(robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * array);

/// Check for srv/AvailableKinematics message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__are_equal(const robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * lhs, const robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * rhs);

/// Copy an array of srv/AvailableKinematics messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence__copy(
  const robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * input,
  robotnik_controllers_msgs__srv__AvailableKinematics_Request__Sequence * output);

/// Initialize srv/AvailableKinematics message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response
 * )) before or use
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Response__init(robotnik_controllers_msgs__srv__AvailableKinematics_Response * msg);

/// Finalize srv/AvailableKinematics message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Response__fini(robotnik_controllers_msgs__srv__AvailableKinematics_Response * msg);

/// Create srv/AvailableKinematics message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
robotnik_controllers_msgs__srv__AvailableKinematics_Response *
robotnik_controllers_msgs__srv__AvailableKinematics_Response__create();

/// Destroy srv/AvailableKinematics message.
/**
 * It calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Response__destroy(robotnik_controllers_msgs__srv__AvailableKinematics_Response * msg);

/// Check for srv/AvailableKinematics message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Response__are_equal(const robotnik_controllers_msgs__srv__AvailableKinematics_Response * lhs, const robotnik_controllers_msgs__srv__AvailableKinematics_Response * rhs);

/// Copy a srv/AvailableKinematics message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Response__copy(
  const robotnik_controllers_msgs__srv__AvailableKinematics_Response * input,
  robotnik_controllers_msgs__srv__AvailableKinematics_Response * output);

/// Initialize array of srv/AvailableKinematics messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__init(robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * array, size_t size);

/// Finalize array of srv/AvailableKinematics messages.
/**
 * It calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__fini(robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * array);

/// Create array of srv/AvailableKinematics messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence *
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__create(size_t size);

/// Destroy array of srv/AvailableKinematics messages.
/**
 * It calls
 * robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
void
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__destroy(robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * array);

/// Check for srv/AvailableKinematics message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__are_equal(const robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * lhs, const robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * rhs);

/// Copy an array of srv/AvailableKinematics messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_controllers_msgs
bool
robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence__copy(
  const robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * input,
  robotnik_controllers_msgs__srv__AvailableKinematics_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_CONTROLLERS_MSGS__SRV__DETAIL__AVAILABLE_KINEMATICS__FUNCTIONS_H_
