// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotnik_perception_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice
#include "robotnik_perception_msgs/msg/detail/detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `objects`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
robotnik_perception_msgs__msg__DetectionArray__init(robotnik_perception_msgs__msg__DetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robotnik_perception_msgs__msg__DetectionArray__fini(msg);
    return false;
  }
  // objects
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->objects, 0)) {
    robotnik_perception_msgs__msg__DetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
robotnik_perception_msgs__msg__DetectionArray__fini(robotnik_perception_msgs__msg__DetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // objects
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->objects);
}

bool
robotnik_perception_msgs__msg__DetectionArray__are_equal(const robotnik_perception_msgs__msg__DetectionArray * lhs, const robotnik_perception_msgs__msg__DetectionArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // objects
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  return true;
}

bool
robotnik_perception_msgs__msg__DetectionArray__copy(
  const robotnik_perception_msgs__msg__DetectionArray * input,
  robotnik_perception_msgs__msg__DetectionArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // objects
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  return true;
}

robotnik_perception_msgs__msg__DetectionArray *
robotnik_perception_msgs__msg__DetectionArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_perception_msgs__msg__DetectionArray * msg = (robotnik_perception_msgs__msg__DetectionArray *)allocator.allocate(sizeof(robotnik_perception_msgs__msg__DetectionArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotnik_perception_msgs__msg__DetectionArray));
  bool success = robotnik_perception_msgs__msg__DetectionArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotnik_perception_msgs__msg__DetectionArray__destroy(robotnik_perception_msgs__msg__DetectionArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotnik_perception_msgs__msg__DetectionArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotnik_perception_msgs__msg__DetectionArray__Sequence__init(robotnik_perception_msgs__msg__DetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_perception_msgs__msg__DetectionArray * data = NULL;

  if (size) {
    data = (robotnik_perception_msgs__msg__DetectionArray *)allocator.zero_allocate(size, sizeof(robotnik_perception_msgs__msg__DetectionArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotnik_perception_msgs__msg__DetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotnik_perception_msgs__msg__DetectionArray__fini(&data[i - 1]);
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
robotnik_perception_msgs__msg__DetectionArray__Sequence__fini(robotnik_perception_msgs__msg__DetectionArray__Sequence * array)
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
      robotnik_perception_msgs__msg__DetectionArray__fini(&array->data[i]);
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

robotnik_perception_msgs__msg__DetectionArray__Sequence *
robotnik_perception_msgs__msg__DetectionArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_perception_msgs__msg__DetectionArray__Sequence * array = (robotnik_perception_msgs__msg__DetectionArray__Sequence *)allocator.allocate(sizeof(robotnik_perception_msgs__msg__DetectionArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotnik_perception_msgs__msg__DetectionArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotnik_perception_msgs__msg__DetectionArray__Sequence__destroy(robotnik_perception_msgs__msg__DetectionArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotnik_perception_msgs__msg__DetectionArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotnik_perception_msgs__msg__DetectionArray__Sequence__are_equal(const robotnik_perception_msgs__msg__DetectionArray__Sequence * lhs, const robotnik_perception_msgs__msg__DetectionArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotnik_perception_msgs__msg__DetectionArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotnik_perception_msgs__msg__DetectionArray__Sequence__copy(
  const robotnik_perception_msgs__msg__DetectionArray__Sequence * input,
  robotnik_perception_msgs__msg__DetectionArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotnik_perception_msgs__msg__DetectionArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotnik_perception_msgs__msg__DetectionArray * data =
      (robotnik_perception_msgs__msg__DetectionArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotnik_perception_msgs__msg__DetectionArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotnik_perception_msgs__msg__DetectionArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotnik_perception_msgs__msg__DetectionArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
