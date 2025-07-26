// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_navigation_msgs:action/Dock.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__DOCK__STRUCT_H_
#define ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__DOCK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'dock_frame'
// Member 'robot_dock_frame'
#include "rosidl_runtime_c/string.h"
// Member 'dock_offset'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"
// Member 'maximum_velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_Goal
{
  rosidl_runtime_c__String dock_frame;
  rosidl_runtime_c__String robot_dock_frame;
  geometry_msgs__msg__Pose2D dock_offset;
  geometry_msgs__msg__Twist maximum_velocity;
} robotnik_navigation_msgs__action__Dock_Goal;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_Goal.
typedef struct robotnik_navigation_msgs__action__Dock_Goal__Sequence
{
  robotnik_navigation_msgs__action__Dock_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_Result
{
  robotnik_common_msgs__msg__Response response;
} robotnik_navigation_msgs__action__Dock_Result;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_Result.
typedef struct robotnik_navigation_msgs__action__Dock_Result__Sequence
{
  robotnik_navigation_msgs__action__Dock_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'remaining'
// already included above
// #include "geometry_msgs/msg/detail/pose2_d__struct.h"
// Member 'current_velocity'
// already included above
// #include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_Feedback
{
  geometry_msgs__msg__Pose2D remaining;
  geometry_msgs__msg__Twist current_velocity;
} robotnik_navigation_msgs__action__Dock_Feedback;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_Feedback.
typedef struct robotnik_navigation_msgs__action__Dock_Feedback__Sequence
{
  robotnik_navigation_msgs__action__Dock_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robotnik_navigation_msgs/action/detail/dock__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robotnik_navigation_msgs__action__Dock_Goal goal;
} robotnik_navigation_msgs__action__Dock_SendGoal_Request;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_SendGoal_Request.
typedef struct robotnik_navigation_msgs__action__Dock_SendGoal_Request__Sequence
{
  robotnik_navigation_msgs__action__Dock_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robotnik_navigation_msgs__action__Dock_SendGoal_Response;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_SendGoal_Response.
typedef struct robotnik_navigation_msgs__action__Dock_SendGoal_Response__Sequence
{
  robotnik_navigation_msgs__action__Dock_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robotnik_navigation_msgs__action__Dock_GetResult_Request;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_GetResult_Request.
typedef struct robotnik_navigation_msgs__action__Dock_GetResult_Request__Sequence
{
  robotnik_navigation_msgs__action__Dock_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robotnik_navigation_msgs/action/detail/dock__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_GetResult_Response
{
  int8_t status;
  robotnik_navigation_msgs__action__Dock_Result result;
} robotnik_navigation_msgs__action__Dock_GetResult_Response;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_GetResult_Response.
typedef struct robotnik_navigation_msgs__action__Dock_GetResult_Response__Sequence
{
  robotnik_navigation_msgs__action__Dock_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robotnik_navigation_msgs/action/detail/dock__struct.h"

/// Struct defined in action/Dock in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Dock_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robotnik_navigation_msgs__action__Dock_Feedback feedback;
} robotnik_navigation_msgs__action__Dock_FeedbackMessage;

// Struct for a sequence of robotnik_navigation_msgs__action__Dock_FeedbackMessage.
typedef struct robotnik_navigation_msgs__action__Dock_FeedbackMessage__Sequence
{
  robotnik_navigation_msgs__action__Dock_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Dock_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__DOCK__STRUCT_H_
