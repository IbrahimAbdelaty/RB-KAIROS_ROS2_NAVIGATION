// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_battery_msgs:msg/BatteryStatusStamped.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__STRUCT_H_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'status'
#include "robotnik_battery_msgs/msg/detail/battery_status__struct.h"

/// Struct defined in msg/BatteryStatusStamped in the package robotnik_battery_msgs.
typedef struct robotnik_battery_msgs__msg__BatteryStatusStamped
{
  std_msgs__msg__Header header;
  robotnik_battery_msgs__msg__BatteryStatus status;
} robotnik_battery_msgs__msg__BatteryStatusStamped;

// Struct for a sequence of robotnik_battery_msgs__msg__BatteryStatusStamped.
typedef struct robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence
{
  robotnik_battery_msgs__msg__BatteryStatusStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_battery_msgs__msg__BatteryStatusStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS_STAMPED__STRUCT_H_
