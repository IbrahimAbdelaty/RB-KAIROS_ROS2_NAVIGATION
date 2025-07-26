// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotnik_battery_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice
#include "robotnik_battery_msgs/msg/detail/battery_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `cell_voltages`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
robotnik_battery_msgs__msg__BatteryStatus__init(robotnik_battery_msgs__msg__BatteryStatus * msg)
{
  if (!msg) {
    return false;
  }
  // voltage
  // current
  // level
  // time_remaining
  // time_charging
  // is_charging
  // cell_voltages
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cell_voltages, 0)) {
    robotnik_battery_msgs__msg__BatteryStatus__fini(msg);
    return false;
  }
  return true;
}

void
robotnik_battery_msgs__msg__BatteryStatus__fini(robotnik_battery_msgs__msg__BatteryStatus * msg)
{
  if (!msg) {
    return;
  }
  // voltage
  // current
  // level
  // time_remaining
  // time_charging
  // is_charging
  // cell_voltages
  rosidl_runtime_c__float__Sequence__fini(&msg->cell_voltages);
}

bool
robotnik_battery_msgs__msg__BatteryStatus__are_equal(const robotnik_battery_msgs__msg__BatteryStatus * lhs, const robotnik_battery_msgs__msg__BatteryStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // level
  if (lhs->level != rhs->level) {
    return false;
  }
  // time_remaining
  if (lhs->time_remaining != rhs->time_remaining) {
    return false;
  }
  // time_charging
  if (lhs->time_charging != rhs->time_charging) {
    return false;
  }
  // is_charging
  if (lhs->is_charging != rhs->is_charging) {
    return false;
  }
  // cell_voltages
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->cell_voltages), &(rhs->cell_voltages)))
  {
    return false;
  }
  return true;
}

bool
robotnik_battery_msgs__msg__BatteryStatus__copy(
  const robotnik_battery_msgs__msg__BatteryStatus * input,
  robotnik_battery_msgs__msg__BatteryStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // voltage
  output->voltage = input->voltage;
  // current
  output->current = input->current;
  // level
  output->level = input->level;
  // time_remaining
  output->time_remaining = input->time_remaining;
  // time_charging
  output->time_charging = input->time_charging;
  // is_charging
  output->is_charging = input->is_charging;
  // cell_voltages
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->cell_voltages), &(output->cell_voltages)))
  {
    return false;
  }
  return true;
}

robotnik_battery_msgs__msg__BatteryStatus *
robotnik_battery_msgs__msg__BatteryStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_battery_msgs__msg__BatteryStatus * msg = (robotnik_battery_msgs__msg__BatteryStatus *)allocator.allocate(sizeof(robotnik_battery_msgs__msg__BatteryStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotnik_battery_msgs__msg__BatteryStatus));
  bool success = robotnik_battery_msgs__msg__BatteryStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotnik_battery_msgs__msg__BatteryStatus__destroy(robotnik_battery_msgs__msg__BatteryStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotnik_battery_msgs__msg__BatteryStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotnik_battery_msgs__msg__BatteryStatus__Sequence__init(robotnik_battery_msgs__msg__BatteryStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_battery_msgs__msg__BatteryStatus * data = NULL;

  if (size) {
    data = (robotnik_battery_msgs__msg__BatteryStatus *)allocator.zero_allocate(size, sizeof(robotnik_battery_msgs__msg__BatteryStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotnik_battery_msgs__msg__BatteryStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotnik_battery_msgs__msg__BatteryStatus__fini(&data[i - 1]);
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
robotnik_battery_msgs__msg__BatteryStatus__Sequence__fini(robotnik_battery_msgs__msg__BatteryStatus__Sequence * array)
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
      robotnik_battery_msgs__msg__BatteryStatus__fini(&array->data[i]);
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

robotnik_battery_msgs__msg__BatteryStatus__Sequence *
robotnik_battery_msgs__msg__BatteryStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotnik_battery_msgs__msg__BatteryStatus__Sequence * array = (robotnik_battery_msgs__msg__BatteryStatus__Sequence *)allocator.allocate(sizeof(robotnik_battery_msgs__msg__BatteryStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotnik_battery_msgs__msg__BatteryStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotnik_battery_msgs__msg__BatteryStatus__Sequence__destroy(robotnik_battery_msgs__msg__BatteryStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotnik_battery_msgs__msg__BatteryStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotnik_battery_msgs__msg__BatteryStatus__Sequence__are_equal(const robotnik_battery_msgs__msg__BatteryStatus__Sequence * lhs, const robotnik_battery_msgs__msg__BatteryStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotnik_battery_msgs__msg__BatteryStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotnik_battery_msgs__msg__BatteryStatus__Sequence__copy(
  const robotnik_battery_msgs__msg__BatteryStatus__Sequence * input,
  robotnik_battery_msgs__msg__BatteryStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotnik_battery_msgs__msg__BatteryStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotnik_battery_msgs__msg__BatteryStatus * data =
      (robotnik_battery_msgs__msg__BatteryStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotnik_battery_msgs__msg__BatteryStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotnik_battery_msgs__msg__BatteryStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotnik_battery_msgs__msg__BatteryStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
