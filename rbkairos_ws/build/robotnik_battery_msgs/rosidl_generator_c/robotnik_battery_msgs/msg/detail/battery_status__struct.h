// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_battery_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cell_voltages'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BatteryStatus in the package robotnik_battery_msgs.
typedef struct robotnik_battery_msgs__msg__BatteryStatus
{
  /// in volts
  float voltage;
  /// in amperes
  float current;
  /// in %
  float level;
  /// in minutes
  uint32_t time_remaining;
  /// in minutes
  uint32_t time_charging;
  /// true when connected
  bool is_charging;
  rosidl_runtime_c__float__Sequence cell_voltages;
} robotnik_battery_msgs__msg__BatteryStatus;

// Struct for a sequence of robotnik_battery_msgs__msg__BatteryStatus.
typedef struct robotnik_battery_msgs__msg__BatteryStatus__Sequence
{
  robotnik_battery_msgs__msg__BatteryStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_battery_msgs__msg__BatteryStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
