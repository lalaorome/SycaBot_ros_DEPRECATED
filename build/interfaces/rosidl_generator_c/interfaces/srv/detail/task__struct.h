// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/Task.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TASK__STRUCT_H_
#define INTERFACES__SRV__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/Task in the package interfaces.
typedef struct interfaces__srv__Task_Request
{
  int64_t id;
} interfaces__srv__Task_Request;

// Struct for a sequence of interfaces__srv__Task_Request.
typedef struct interfaces__srv__Task_Request__Sequence
{
  interfaces__srv__Task_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__Task_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'task'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in srv/Task in the package interfaces.
typedef struct interfaces__srv__Task_Response
{
  geometry_msgs__msg__Point task;
} interfaces__srv__Task_Response;

// Struct for a sequence of interfaces__srv__Task_Response.
typedef struct interfaces__srv__Task_Response__Sequence
{
  interfaces__srv__Task_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__Task_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__TASK__STRUCT_H_
