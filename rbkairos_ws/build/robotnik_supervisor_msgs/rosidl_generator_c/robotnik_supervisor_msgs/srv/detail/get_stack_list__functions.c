// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotnik_supervisor_msgs:srv/GetStackList.idl
// generated code does not contain a copyright notice
#include "robotnik_supervisor_msgs/srv/detail/get_stack_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
robotnik_supervisor_msgs__srv__GetStackList_Request__init(robotnik_supervisor_msgs__srv__GetStackList_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Request__fini(robotnik_supervisor_msgs__srv__GetStackList_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Request__are_equal(const robotnik_supervisor_msgs__srv__GetStackList_Request * lhs, const robotnik_supervisor_msgs__srv__GetStackList_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Request__copy(
  const robotnik_supervisor_msgs__srv__GetStackList_Request * input,
  robotnik_supervisor_msgs__srv__GetStackList_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

robotnik_supervisor_msgs__srv__GetStackList_Request *
robotnik_supervisor_msgs__srv__GetStackList_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_supervisor_msgs__srv__GetStackList_Request * msg = (robotnik_supervisor_msgs__srv__GetStackList_Request *)allocator.allocate(sizeof(robotnik_supervisor_msgs__srv__GetStackList_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotnik_supervisor_msgs__srv__GetStackList_Request));
  bool success = robotnik_supervisor_msgs__srv__GetStackList_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Request__destroy(robotnik_supervisor_msgs__srv__GetStackList_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotnik_supervisor_msgs__srv__GetStackList_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__init(robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_supervisor_msgs__srv__GetStackList_Request * data = NULL;

  if (size) {
    data = (robotnik_supervisor_msgs__srv__GetStackList_Request *)allocator.zero_allocate(size, sizeof(robotnik_supervisor_msgs__srv__GetStackList_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotnik_supervisor_msgs__srv__GetStackList_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotnik_supervisor_msgs__srv__GetStackList_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__fini(robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robotnik_supervisor_msgs__srv__GetStackList_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence *
robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * array = (robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence *)allocator.allocate(sizeof(robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__destroy(robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__are_equal(const robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * lhs, const robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotnik_supervisor_msgs__srv__GetStackList_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence__copy(
  const robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * input,
  robotnik_supervisor_msgs__srv__GetStackList_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotnik_supervisor_msgs__srv__GetStackList_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotnik_supervisor_msgs__srv__GetStackList_Request * data =
      (robotnik_supervisor_msgs__srv__GetStackList_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotnik_supervisor_msgs__srv__GetStackList_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotnik_supervisor_msgs__srv__GetStackList_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotnik_supervisor_msgs__srv__GetStackList_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `response`
#include "robotnik_common_msgs/msg/detail/response__functions.h"
// Member `stacks`
#include "rosidl_runtime_c/string_functions.h"

bool
robotnik_supervisor_msgs__srv__GetStackList_Response__init(robotnik_supervisor_msgs__srv__GetStackList_Response * msg)
{
  if (!msg) {
    return false;
  }
  // response
  if (!robotnik_common_msgs__msg__Response__init(&msg->response)) {
    robotnik_supervisor_msgs__srv__GetStackList_Response__fini(msg);
    return false;
  }
  // stacks
  if (!rosidl_runtime_c__String__Sequence__init(&msg->stacks, 0)) {
    robotnik_supervisor_msgs__srv__GetStackList_Response__fini(msg);
    return false;
  }
  return true;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Response__fini(robotnik_supervisor_msgs__srv__GetStackList_Response * msg)
{
  if (!msg) {
    return;
  }
  // response
  robotnik_common_msgs__msg__Response__fini(&msg->response);
  // stacks
  rosidl_runtime_c__String__Sequence__fini(&msg->stacks);
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Response__are_equal(const robotnik_supervisor_msgs__srv__GetStackList_Response * lhs, const robotnik_supervisor_msgs__srv__GetStackList_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // response
  if (!robotnik_common_msgs__msg__Response__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  // stacks
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->stacks), &(rhs->stacks)))
  {
    return false;
  }
  return true;
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Response__copy(
  const robotnik_supervisor_msgs__srv__GetStackList_Response * input,
  robotnik_supervisor_msgs__srv__GetStackList_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // response
  if (!robotnik_common_msgs__msg__Response__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  // stacks
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->stacks), &(output->stacks)))
  {
    return false;
  }
  return true;
}

robotnik_supervisor_msgs__srv__GetStackList_Response *
robotnik_supervisor_msgs__srv__GetStackList_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_supervisor_msgs__srv__GetStackList_Response * msg = (robotnik_supervisor_msgs__srv__GetStackList_Response *)allocator.allocate(sizeof(robotnik_supervisor_msgs__srv__GetStackList_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotnik_supervisor_msgs__srv__GetStackList_Response));
  bool success = robotnik_supervisor_msgs__srv__GetStackList_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Response__destroy(robotnik_supervisor_msgs__srv__GetStackList_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotnik_supervisor_msgs__srv__GetStackList_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__init(robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_supervisor_msgs__srv__GetStackList_Response * data = NULL;

  if (size) {
    data = (robotnik_supervisor_msgs__srv__GetStackList_Response *)allocator.zero_allocate(size, sizeof(robotnik_supervisor_msgs__srv__GetStackList_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotnik_supervisor_msgs__srv__GetStackList_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotnik_supervisor_msgs__srv__GetStackList_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__fini(robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robotnik_supervisor_msgs__srv__GetStackList_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence *
robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * array = (robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence *)allocator.allocate(sizeof(robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__destroy(robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__are_equal(const robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * lhs, const robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotnik_supervisor_msgs__srv__GetStackList_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence__copy(
  const robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * input,
  robotnik_supervisor_msgs__srv__GetStackList_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotnik_supervisor_msgs__srv__GetStackList_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotnik_supervisor_msgs__srv__GetStackList_Response * data =
      (robotnik_supervisor_msgs__srv__GetStackList_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotnik_supervisor_msgs__srv__GetStackList_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotnik_supervisor_msgs__srv__GetStackList_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotnik_supervisor_msgs__srv__GetStackList_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
