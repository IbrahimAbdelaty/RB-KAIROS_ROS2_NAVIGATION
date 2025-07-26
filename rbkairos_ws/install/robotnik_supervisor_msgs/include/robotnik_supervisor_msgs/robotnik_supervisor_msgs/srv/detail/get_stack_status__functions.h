// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotnik_supervisor_msgs:srv/GetStackStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_STACK_STATUS__FUNCTIONS_H_
#define ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_STACK_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotnik_supervisor_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robotnik_supervisor_msgs/srv/detail/get_stack_status__struct.h"

/// Initialize srv/GetStackStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request
 * )) before or use
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Request__init(robotnik_supervisor_msgs__srv__GetStackStatus_Request * msg);

/// Finalize srv/GetStackStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Request__fini(robotnik_supervisor_msgs__srv__GetStackStatus_Request * msg);

/// Create srv/GetStackStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
robotnik_supervisor_msgs__srv__GetStackStatus_Request *
robotnik_supervisor_msgs__srv__GetStackStatus_Request__create();

/// Destroy srv/GetStackStatus message.
/**
 * It calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Request__destroy(robotnik_supervisor_msgs__srv__GetStackStatus_Request * msg);

/// Check for srv/GetStackStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Request__are_equal(const robotnik_supervisor_msgs__srv__GetStackStatus_Request * lhs, const robotnik_supervisor_msgs__srv__GetStackStatus_Request * rhs);

/// Copy a srv/GetStackStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Request__copy(
  const robotnik_supervisor_msgs__srv__GetStackStatus_Request * input,
  robotnik_supervisor_msgs__srv__GetStackStatus_Request * output);

/// Initialize array of srv/GetStackStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__init(robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetStackStatus messages.
/**
 * It calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__fini(robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * array);

/// Create array of srv/GetStackStatus messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence *
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetStackStatus messages.
/**
 * It calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__destroy(robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * array);

/// Check for srv/GetStackStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__are_equal(const robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * lhs, const robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * rhs);

/// Copy an array of srv/GetStackStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence__copy(
  const robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * input,
  robotnik_supervisor_msgs__srv__GetStackStatus_Request__Sequence * output);

/// Initialize srv/GetStackStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response
 * )) before or use
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Response__init(robotnik_supervisor_msgs__srv__GetStackStatus_Response * msg);

/// Finalize srv/GetStackStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Response__fini(robotnik_supervisor_msgs__srv__GetStackStatus_Response * msg);

/// Create srv/GetStackStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
robotnik_supervisor_msgs__srv__GetStackStatus_Response *
robotnik_supervisor_msgs__srv__GetStackStatus_Response__create();

/// Destroy srv/GetStackStatus message.
/**
 * It calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Response__destroy(robotnik_supervisor_msgs__srv__GetStackStatus_Response * msg);

/// Check for srv/GetStackStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Response__are_equal(const robotnik_supervisor_msgs__srv__GetStackStatus_Response * lhs, const robotnik_supervisor_msgs__srv__GetStackStatus_Response * rhs);

/// Copy a srv/GetStackStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Response__copy(
  const robotnik_supervisor_msgs__srv__GetStackStatus_Response * input,
  robotnik_supervisor_msgs__srv__GetStackStatus_Response * output);

/// Initialize array of srv/GetStackStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__init(robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetStackStatus messages.
/**
 * It calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__fini(robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * array);

/// Create array of srv/GetStackStatus messages.
/**
 * It allocates the memory for the array and calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence *
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetStackStatus messages.
/**
 * It calls
 * robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
void
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__destroy(robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * array);

/// Check for srv/GetStackStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__are_equal(const robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * lhs, const robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * rhs);

/// Copy an array of srv/GetStackStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotnik_supervisor_msgs
bool
robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence__copy(
  const robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * input,
  robotnik_supervisor_msgs__srv__GetStackStatus_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_SUPERVISOR_MSGS__SRV__DETAIL__GET_STACK_STATUS__FUNCTIONS_H_
