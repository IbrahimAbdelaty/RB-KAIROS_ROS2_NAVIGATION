// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_battery_msgs:msg/DockingStationStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__STRUCT_H_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_DISABLED'.
static const char * const robotnik_battery_msgs__msg__DockingStationStatus__MODE_DISABLED = "disabled";

/// Constant 'MODE_AUTO_HW'.
/**
  * Unattended relay detection & activation with no inputs/outputs feedback. Done by the hw
 */
static const char * const robotnik_battery_msgs__msg__DockingStationStatus__MODE_AUTO_HW = "automatic_hw";

/// Constant 'MODE_AUTO_SW'.
/**
  * Unattended relay detection & activation with inputs/outputs feedback. Done by the sw
 */
static const char * const robotnik_battery_msgs__msg__DockingStationStatus__MODE_AUTO_SW = "automatic_sw";

/// Constant 'MODE_MANUAL_SW'.
/**
  * Unattended relay detection & and manual activation of the charging relay
 */
static const char * const robotnik_battery_msgs__msg__DockingStationStatus__MODE_MANUAL_SW = "manual_sw";

// Include directives for member types
// Member 'operation_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DockingStationStatus in the package robotnik_battery_msgs.
/**
  * Modes of operation:
  * no docking station contacts
 */
typedef struct robotnik_battery_msgs__msg__DockingStationStatus
{
  rosidl_runtime_c__String operation_mode;
  /// shows if there's contact with the charger
  bool contact_relay_status;
  /// shows if the relay for the charge is active or not
  bool charger_relay_status;
} robotnik_battery_msgs__msg__DockingStationStatus;

// Struct for a sequence of robotnik_battery_msgs__msg__DockingStationStatus.
typedef struct robotnik_battery_msgs__msg__DockingStationStatus__Sequence
{
  robotnik_battery_msgs__msg__DockingStationStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_battery_msgs__msg__DockingStationStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__DOCKING_STATION_STATUS__STRUCT_H_
