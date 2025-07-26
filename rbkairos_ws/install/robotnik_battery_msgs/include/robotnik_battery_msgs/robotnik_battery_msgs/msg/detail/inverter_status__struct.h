// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_battery_msgs:msg/InverterStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__STRUCT_H_
#define ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'serial_number'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/InverterStatus in the package robotnik_battery_msgs.
typedef struct robotnik_battery_msgs__msg__InverterStatus
{
  /// Output Voltage in Volts (Mandatory)
  float ac_voltage;
  /// Input Voltage in Volts (If unmeasured NaN)
  float dc_voltage;
  /// Current percentage load on 0 to 1 range (If unmeasured NaN)
  float load;
  /// Charge percentage on 0 to 1 range (If unmeasured NaN)
  float percentage;
  /// Current temperature in centigrads (If unmeasured NaN)
  float temperature;
  /// True if the inverter is on
  bool on;
  /// The best approximation of the battery serial number
  rosidl_runtime_c__String serial_number;
} robotnik_battery_msgs__msg__InverterStatus;

// Struct for a sequence of robotnik_battery_msgs__msg__InverterStatus.
typedef struct robotnik_battery_msgs__msg__InverterStatus__Sequence
{
  robotnik_battery_msgs__msg__InverterStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_battery_msgs__msg__InverterStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_BATTERY_MSGS__MSG__DETAIL__INVERTER_STATUS__STRUCT_H_
