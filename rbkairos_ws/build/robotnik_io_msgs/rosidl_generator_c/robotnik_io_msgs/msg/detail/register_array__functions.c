// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotnik_io_msgs:msg/RegisterArray.idl
// generated code does not contain a copyright notice
#include "robotnik_io_msgs/msg/detail/register_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `registers`
#include "robotnik_io_msgs/msg/detail/register__functions.h"

bool
robotnik_io_msgs__msg__RegisterArray__init(robotnik_io_msgs__msg__RegisterArray * msg)
{
  if (!msg) {
    return false;
  }
  // registers
  if (!robotnik_io_msgs__msg__Register__Sequence__init(&msg->registers, 0)) {
    robotnik_io_msgs__msg__RegisterArray__fini(msg);
    return false;
  }
  return true;
}

void
robotnik_io_msgs__msg__RegisterArray__fini(robotnik_io_msgs__msg__RegisterArray * msg)
{
  if (!msg) {
    return;
  }
  // registers
  robotnik_io_msgs__msg__Register__Sequence__fini(&msg->registers);
}

bool
robotnik_io_msgs__msg__RegisterArray__are_equal(const robotnik_io_msgs__msg__RegisterArray * lhs, const robotnik_io_msgs__msg__RegisterArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // registers
  if (!robotnik_io_msgs__msg__Register__Sequence__are_equal(
      &(lhs->registers), &(rhs->registers)))
  {
    return false;
  }
  return true;
}

bool
robotnik_io_msgs__msg__RegisterArray__copy(
  const robotnik_io_msgs__msg__RegisterArray * input,
  robotnik_io_msgs__msg__RegisterArray * output)
{
  if (!input || !output) {
    return false;
  }
  // registers
  if (!robotnik_io_msgs__msg__Register__Sequence__copy(
      &(input->registers), &(output->registers)))
  {
    return false;
  }
  return true;
}

robotnik_io_msgs__msg__RegisterArray *
robotnik_io_msgs__msg__RegisterArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_io_msgs__msg__RegisterArray * msg = (robotnik_io_msgs__msg__RegisterArray *)allocator.allocate(sizeof(robotnik_io_msgs__msg__RegisterArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotnik_io_msgs__msg__RegisterArray));
  bool success = robotnik_io_msgs__msg__RegisterArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotnik_io_msgs__msg__RegisterArray__destroy(robotnik_io_msgs__msg__RegisterArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotnik_io_msgs__msg__RegisterArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotnik_io_msgs__msg__RegisterArray__Sequence__init(robotnik_io_msgs__msg__RegisterArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_io_msgs__msg__RegisterArray * data = NULL;

  if (size) {
    data = (robotnik_io_msgs__msg__RegisterArray *)allocator.zero_allocate(size, sizeof(robotnik_io_msgs__msg__RegisterArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotnik_io_msgs__msg__RegisterArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotnik_io_msgs__msg__RegisterArray__fini(&data[i - 1]);
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
robotnik_io_msgs__msg__RegisterArray__Sequence__fini(robotnik_io_msgs__msg__RegisterArray__Sequence * array)
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
      robotnik_io_msgs__msg__RegisterArray__fini(&array->data[i]);
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

robotnik_io_msgs__msg__RegisterArray__Sequence *
robotnik_io_msgs__msg__RegisterArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_io_msgs__msg__RegisterArray__Sequence * array = (robotnik_io_msgs__msg__RegisterArray__Sequence *)allocator.allocate(sizeof(robotnik_io_msgs__msg__RegisterArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotnik_io_msgs__msg__RegisterArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotnik_io_msgs__msg__RegisterArray__Sequence__destroy(robotnik_io_msgs__msg__RegisterArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotnik_io_msgs__msg__RegisterArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotnik_io_msgs__msg__RegisterArray__Sequence__are_equal(const robotnik_io_msgs__msg__RegisterArray__Sequence * lhs, const robotnik_io_msgs__msg__RegisterArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotnik_io_msgs__msg__RegisterArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotnik_io_msgs__msg__RegisterArray__Sequence__copy(
  const robotnik_io_msgs__msg__RegisterArray__Sequence * input,
  robotnik_io_msgs__msg__RegisterArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotnik_io_msgs__msg__RegisterArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotnik_io_msgs__msg__RegisterArray * data =
      (robotnik_io_msgs__msg__RegisterArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotnik_io_msgs__msg__RegisterArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotnik_io_msgs__msg__RegisterArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotnik_io_msgs__msg__RegisterArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
