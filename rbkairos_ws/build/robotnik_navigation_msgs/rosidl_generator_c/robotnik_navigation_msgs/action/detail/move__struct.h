// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotnik_navigation_msgs:action/Move.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__MOVE__STRUCT_H_
#define ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'goal'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"
// Member 'maximum_velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_Goal
{
  geometry_msgs__msg__Pose2D goal;
  geometry_msgs__msg__Twist maximum_velocity;
} robotnik_navigation_msgs__action__Move_Goal;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_Goal.
typedef struct robotnik_navigation_msgs__action__Move_Goal__Sequence
{
  robotnik_navigation_msgs__action__Move_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
#include "robotnik_common_msgs/msg/detail/response__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_Result
{
  robotnik_common_msgs__msg__Response response;
} robotnik_navigation_msgs__action__Move_Result;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_Result.
typedef struct robotnik_navigation_msgs__action__Move_Result__Sequence
{
  robotnik_navigation_msgs__action__Move_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'remaining'
// already included above
// #include "geometry_msgs/msg/detail/pose2_d__struct.h"
// Member 'current_velocity'
// already included above
// #include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_Feedback
{
  geometry_msgs__msg__Pose2D remaining;
  geometry_msgs__msg__Twist current_velocity;
} robotnik_navigation_msgs__action__Move_Feedback;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_Feedback.
typedef struct robotnik_navigation_msgs__action__Move_Feedback__Sequence
{
  robotnik_navigation_msgs__action__Move_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robotnik_navigation_msgs/action/detail/move__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robotnik_navigation_msgs__action__Move_Goal goal;
} robotnik_navigation_msgs__action__Move_SendGoal_Request;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_SendGoal_Request.
typedef struct robotnik_navigation_msgs__action__Move_SendGoal_Request__Sequence
{
  robotnik_navigation_msgs__action__Move_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robotnik_navigation_msgs__action__Move_SendGoal_Response;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_SendGoal_Response.
typedef struct robotnik_navigation_msgs__action__Move_SendGoal_Response__Sequence
{
  robotnik_navigation_msgs__action__Move_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robotnik_navigation_msgs__action__Move_GetResult_Request;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_GetResult_Request.
typedef struct robotnik_navigation_msgs__action__Move_GetResult_Request__Sequence
{
  robotnik_navigation_msgs__action__Move_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robotnik_navigation_msgs/action/detail/move__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_GetResult_Response
{
  int8_t status;
  robotnik_navigation_msgs__action__Move_Result result;
} robotnik_navigation_msgs__action__Move_GetResult_Response;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_GetResult_Response.
typedef struct robotnik_navigation_msgs__action__Move_GetResult_Response__Sequence
{
  robotnik_navigation_msgs__action__Move_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robotnik_navigation_msgs/action/detail/move__struct.h"

/// Struct defined in action/Move in the package robotnik_navigation_msgs.
typedef struct robotnik_navigation_msgs__action__Move_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robotnik_navigation_msgs__action__Move_Feedback feedback;
} robotnik_navigation_msgs__action__Move_FeedbackMessage;

// Struct for a sequence of robotnik_navigation_msgs__action__Move_FeedbackMessage.
typedef struct robotnik_navigation_msgs__action__Move_FeedbackMessage__Sequence
{
  robotnik_navigation_msgs__action__Move_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotnik_navigation_msgs__action__Move_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__MOVE__STRUCT_H_
