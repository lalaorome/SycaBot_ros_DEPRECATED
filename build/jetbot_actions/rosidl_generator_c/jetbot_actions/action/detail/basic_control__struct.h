// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jetbot_actions:action/BasicControl.idl
// generated code does not contain a copyright notice

#ifndef JETBOT_ACTIONS__ACTION__DETAIL__BASIC_CONTROL__STRUCT_H_
#define JETBOT_ACTIONS__ACTION__DETAIL__BASIC_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'order'
#include "rosidl_runtime_c/string.h"

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_Goal
{
  rosidl_runtime_c__String order;
} jetbot_actions__action__BasicControl_Goal;

// Struct for a sequence of jetbot_actions__action__BasicControl_Goal.
typedef struct jetbot_actions__action__BasicControl_Goal__Sequence
{
  jetbot_actions__action__BasicControl_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_Result
{
  bool achieved;
} jetbot_actions__action__BasicControl_Result;

// Struct for a sequence of jetbot_actions__action__BasicControl_Result.
typedef struct jetbot_actions__action__BasicControl_Result__Sequence
{
  jetbot_actions__action__BasicControl_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_Result__Sequence;


// Constants defined in the message

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} jetbot_actions__action__BasicControl_Feedback;

// Struct for a sequence of jetbot_actions__action__BasicControl_Feedback.
typedef struct jetbot_actions__action__BasicControl_Feedback__Sequence
{
  jetbot_actions__action__BasicControl_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "jetbot_actions/action/detail/basic_control__struct.h"

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  jetbot_actions__action__BasicControl_Goal goal;
} jetbot_actions__action__BasicControl_SendGoal_Request;

// Struct for a sequence of jetbot_actions__action__BasicControl_SendGoal_Request.
typedef struct jetbot_actions__action__BasicControl_SendGoal_Request__Sequence
{
  jetbot_actions__action__BasicControl_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} jetbot_actions__action__BasicControl_SendGoal_Response;

// Struct for a sequence of jetbot_actions__action__BasicControl_SendGoal_Response.
typedef struct jetbot_actions__action__BasicControl_SendGoal_Response__Sequence
{
  jetbot_actions__action__BasicControl_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} jetbot_actions__action__BasicControl_GetResult_Request;

// Struct for a sequence of jetbot_actions__action__BasicControl_GetResult_Request.
typedef struct jetbot_actions__action__BasicControl_GetResult_Request__Sequence
{
  jetbot_actions__action__BasicControl_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.h"

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_GetResult_Response
{
  int8_t status;
  jetbot_actions__action__BasicControl_Result result;
} jetbot_actions__action__BasicControl_GetResult_Response;

// Struct for a sequence of jetbot_actions__action__BasicControl_GetResult_Response.
typedef struct jetbot_actions__action__BasicControl_GetResult_Response__Sequence
{
  jetbot_actions__action__BasicControl_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.h"

// Struct defined in action/BasicControl in the package jetbot_actions.
typedef struct jetbot_actions__action__BasicControl_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  jetbot_actions__action__BasicControl_Feedback feedback;
} jetbot_actions__action__BasicControl_FeedbackMessage;

// Struct for a sequence of jetbot_actions__action__BasicControl_FeedbackMessage.
typedef struct jetbot_actions__action__BasicControl_FeedbackMessage__Sequence
{
  jetbot_actions__action__BasicControl_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jetbot_actions__action__BasicControl_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JETBOT_ACTIONS__ACTION__DETAIL__BASIC_CONTROL__STRUCT_H_
