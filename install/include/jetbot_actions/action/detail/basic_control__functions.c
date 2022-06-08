// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jetbot_actions:action/BasicControl.idl
// generated code does not contain a copyright notice
#include "jetbot_actions/action/detail/basic_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `order`
#include "rosidl_runtime_c/string_functions.h"

bool
jetbot_actions__action__BasicControl_Goal__init(jetbot_actions__action__BasicControl_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // order
  if (!rosidl_runtime_c__String__init(&msg->order)) {
    jetbot_actions__action__BasicControl_Goal__fini(msg);
    return false;
  }
  return true;
}

void
jetbot_actions__action__BasicControl_Goal__fini(jetbot_actions__action__BasicControl_Goal * msg)
{
  if (!msg) {
    return;
  }
  // order
  rosidl_runtime_c__String__fini(&msg->order);
}

jetbot_actions__action__BasicControl_Goal *
jetbot_actions__action__BasicControl_Goal__create()
{
  jetbot_actions__action__BasicControl_Goal * msg = (jetbot_actions__action__BasicControl_Goal *)malloc(sizeof(jetbot_actions__action__BasicControl_Goal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_Goal));
  bool success = jetbot_actions__action__BasicControl_Goal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_Goal__destroy(jetbot_actions__action__BasicControl_Goal * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_Goal__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_Goal__Sequence__init(jetbot_actions__action__BasicControl_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_Goal * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_Goal *)calloc(size, sizeof(jetbot_actions__action__BasicControl_Goal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_Goal__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_Goal__Sequence__fini(jetbot_actions__action__BasicControl_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_Goal__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_Goal__Sequence *
jetbot_actions__action__BasicControl_Goal__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_Goal__Sequence * array = (jetbot_actions__action__BasicControl_Goal__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_Goal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_Goal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_Goal__Sequence__destroy(jetbot_actions__action__BasicControl_Goal__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_Goal__Sequence__fini(array);
  }
  free(array);
}


bool
jetbot_actions__action__BasicControl_Result__init(jetbot_actions__action__BasicControl_Result * msg)
{
  if (!msg) {
    return false;
  }
  // achieved
  return true;
}

void
jetbot_actions__action__BasicControl_Result__fini(jetbot_actions__action__BasicControl_Result * msg)
{
  if (!msg) {
    return;
  }
  // achieved
}

jetbot_actions__action__BasicControl_Result *
jetbot_actions__action__BasicControl_Result__create()
{
  jetbot_actions__action__BasicControl_Result * msg = (jetbot_actions__action__BasicControl_Result *)malloc(sizeof(jetbot_actions__action__BasicControl_Result));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_Result));
  bool success = jetbot_actions__action__BasicControl_Result__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_Result__destroy(jetbot_actions__action__BasicControl_Result * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_Result__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_Result__Sequence__init(jetbot_actions__action__BasicControl_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_Result * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_Result *)calloc(size, sizeof(jetbot_actions__action__BasicControl_Result));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_Result__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_Result__Sequence__fini(jetbot_actions__action__BasicControl_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_Result__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_Result__Sequence *
jetbot_actions__action__BasicControl_Result__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_Result__Sequence * array = (jetbot_actions__action__BasicControl_Result__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_Result__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_Result__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_Result__Sequence__destroy(jetbot_actions__action__BasicControl_Result__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_Result__Sequence__fini(array);
  }
  free(array);
}


bool
jetbot_actions__action__BasicControl_Feedback__init(jetbot_actions__action__BasicControl_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
jetbot_actions__action__BasicControl_Feedback__fini(jetbot_actions__action__BasicControl_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

jetbot_actions__action__BasicControl_Feedback *
jetbot_actions__action__BasicControl_Feedback__create()
{
  jetbot_actions__action__BasicControl_Feedback * msg = (jetbot_actions__action__BasicControl_Feedback *)malloc(sizeof(jetbot_actions__action__BasicControl_Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_Feedback));
  bool success = jetbot_actions__action__BasicControl_Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_Feedback__destroy(jetbot_actions__action__BasicControl_Feedback * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_Feedback__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_Feedback__Sequence__init(jetbot_actions__action__BasicControl_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_Feedback * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_Feedback *)calloc(size, sizeof(jetbot_actions__action__BasicControl_Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_Feedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_Feedback__Sequence__fini(jetbot_actions__action__BasicControl_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_Feedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_Feedback__Sequence *
jetbot_actions__action__BasicControl_Feedback__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_Feedback__Sequence * array = (jetbot_actions__action__BasicControl_Feedback__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_Feedback__Sequence__destroy(jetbot_actions__action__BasicControl_Feedback__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_Feedback__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "jetbot_actions/action/detail/basic_control__functions.h"

bool
jetbot_actions__action__BasicControl_SendGoal_Request__init(jetbot_actions__action__BasicControl_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    jetbot_actions__action__BasicControl_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!jetbot_actions__action__BasicControl_Goal__init(&msg->goal)) {
    jetbot_actions__action__BasicControl_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
jetbot_actions__action__BasicControl_SendGoal_Request__fini(jetbot_actions__action__BasicControl_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  jetbot_actions__action__BasicControl_Goal__fini(&msg->goal);
}

jetbot_actions__action__BasicControl_SendGoal_Request *
jetbot_actions__action__BasicControl_SendGoal_Request__create()
{
  jetbot_actions__action__BasicControl_SendGoal_Request * msg = (jetbot_actions__action__BasicControl_SendGoal_Request *)malloc(sizeof(jetbot_actions__action__BasicControl_SendGoal_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_SendGoal_Request));
  bool success = jetbot_actions__action__BasicControl_SendGoal_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_SendGoal_Request__destroy(jetbot_actions__action__BasicControl_SendGoal_Request * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_SendGoal_Request__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_SendGoal_Request__Sequence__init(jetbot_actions__action__BasicControl_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_SendGoal_Request * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_SendGoal_Request *)calloc(size, sizeof(jetbot_actions__action__BasicControl_SendGoal_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_SendGoal_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_SendGoal_Request__Sequence__fini(jetbot_actions__action__BasicControl_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_SendGoal_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_SendGoal_Request__Sequence *
jetbot_actions__action__BasicControl_SendGoal_Request__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_SendGoal_Request__Sequence * array = (jetbot_actions__action__BasicControl_SendGoal_Request__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_SendGoal_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_SendGoal_Request__Sequence__destroy(jetbot_actions__action__BasicControl_SendGoal_Request__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_SendGoal_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
jetbot_actions__action__BasicControl_SendGoal_Response__init(jetbot_actions__action__BasicControl_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    jetbot_actions__action__BasicControl_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
jetbot_actions__action__BasicControl_SendGoal_Response__fini(jetbot_actions__action__BasicControl_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

jetbot_actions__action__BasicControl_SendGoal_Response *
jetbot_actions__action__BasicControl_SendGoal_Response__create()
{
  jetbot_actions__action__BasicControl_SendGoal_Response * msg = (jetbot_actions__action__BasicControl_SendGoal_Response *)malloc(sizeof(jetbot_actions__action__BasicControl_SendGoal_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_SendGoal_Response));
  bool success = jetbot_actions__action__BasicControl_SendGoal_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_SendGoal_Response__destroy(jetbot_actions__action__BasicControl_SendGoal_Response * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_SendGoal_Response__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_SendGoal_Response__Sequence__init(jetbot_actions__action__BasicControl_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_SendGoal_Response * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_SendGoal_Response *)calloc(size, sizeof(jetbot_actions__action__BasicControl_SendGoal_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_SendGoal_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_SendGoal_Response__Sequence__fini(jetbot_actions__action__BasicControl_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_SendGoal_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_SendGoal_Response__Sequence *
jetbot_actions__action__BasicControl_SendGoal_Response__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_SendGoal_Response__Sequence * array = (jetbot_actions__action__BasicControl_SendGoal_Response__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_SendGoal_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_SendGoal_Response__Sequence__destroy(jetbot_actions__action__BasicControl_SendGoal_Response__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_SendGoal_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
jetbot_actions__action__BasicControl_GetResult_Request__init(jetbot_actions__action__BasicControl_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    jetbot_actions__action__BasicControl_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
jetbot_actions__action__BasicControl_GetResult_Request__fini(jetbot_actions__action__BasicControl_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

jetbot_actions__action__BasicControl_GetResult_Request *
jetbot_actions__action__BasicControl_GetResult_Request__create()
{
  jetbot_actions__action__BasicControl_GetResult_Request * msg = (jetbot_actions__action__BasicControl_GetResult_Request *)malloc(sizeof(jetbot_actions__action__BasicControl_GetResult_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_GetResult_Request));
  bool success = jetbot_actions__action__BasicControl_GetResult_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_GetResult_Request__destroy(jetbot_actions__action__BasicControl_GetResult_Request * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_GetResult_Request__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_GetResult_Request__Sequence__init(jetbot_actions__action__BasicControl_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_GetResult_Request * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_GetResult_Request *)calloc(size, sizeof(jetbot_actions__action__BasicControl_GetResult_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_GetResult_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_GetResult_Request__Sequence__fini(jetbot_actions__action__BasicControl_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_GetResult_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_GetResult_Request__Sequence *
jetbot_actions__action__BasicControl_GetResult_Request__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_GetResult_Request__Sequence * array = (jetbot_actions__action__BasicControl_GetResult_Request__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_GetResult_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_GetResult_Request__Sequence__destroy(jetbot_actions__action__BasicControl_GetResult_Request__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_GetResult_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `result`
// already included above
// #include "jetbot_actions/action/detail/basic_control__functions.h"

bool
jetbot_actions__action__BasicControl_GetResult_Response__init(jetbot_actions__action__BasicControl_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!jetbot_actions__action__BasicControl_Result__init(&msg->result)) {
    jetbot_actions__action__BasicControl_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
jetbot_actions__action__BasicControl_GetResult_Response__fini(jetbot_actions__action__BasicControl_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  jetbot_actions__action__BasicControl_Result__fini(&msg->result);
}

jetbot_actions__action__BasicControl_GetResult_Response *
jetbot_actions__action__BasicControl_GetResult_Response__create()
{
  jetbot_actions__action__BasicControl_GetResult_Response * msg = (jetbot_actions__action__BasicControl_GetResult_Response *)malloc(sizeof(jetbot_actions__action__BasicControl_GetResult_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_GetResult_Response));
  bool success = jetbot_actions__action__BasicControl_GetResult_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_GetResult_Response__destroy(jetbot_actions__action__BasicControl_GetResult_Response * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_GetResult_Response__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_GetResult_Response__Sequence__init(jetbot_actions__action__BasicControl_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_GetResult_Response * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_GetResult_Response *)calloc(size, sizeof(jetbot_actions__action__BasicControl_GetResult_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_GetResult_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_GetResult_Response__Sequence__fini(jetbot_actions__action__BasicControl_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_GetResult_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_GetResult_Response__Sequence *
jetbot_actions__action__BasicControl_GetResult_Response__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_GetResult_Response__Sequence * array = (jetbot_actions__action__BasicControl_GetResult_Response__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_GetResult_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_GetResult_Response__Sequence__destroy(jetbot_actions__action__BasicControl_GetResult_Response__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_GetResult_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "jetbot_actions/action/detail/basic_control__functions.h"

bool
jetbot_actions__action__BasicControl_FeedbackMessage__init(jetbot_actions__action__BasicControl_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    jetbot_actions__action__BasicControl_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!jetbot_actions__action__BasicControl_Feedback__init(&msg->feedback)) {
    jetbot_actions__action__BasicControl_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
jetbot_actions__action__BasicControl_FeedbackMessage__fini(jetbot_actions__action__BasicControl_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  jetbot_actions__action__BasicControl_Feedback__fini(&msg->feedback);
}

jetbot_actions__action__BasicControl_FeedbackMessage *
jetbot_actions__action__BasicControl_FeedbackMessage__create()
{
  jetbot_actions__action__BasicControl_FeedbackMessage * msg = (jetbot_actions__action__BasicControl_FeedbackMessage *)malloc(sizeof(jetbot_actions__action__BasicControl_FeedbackMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jetbot_actions__action__BasicControl_FeedbackMessage));
  bool success = jetbot_actions__action__BasicControl_FeedbackMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
jetbot_actions__action__BasicControl_FeedbackMessage__destroy(jetbot_actions__action__BasicControl_FeedbackMessage * msg)
{
  if (msg) {
    jetbot_actions__action__BasicControl_FeedbackMessage__fini(msg);
  }
  free(msg);
}


bool
jetbot_actions__action__BasicControl_FeedbackMessage__Sequence__init(jetbot_actions__action__BasicControl_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  jetbot_actions__action__BasicControl_FeedbackMessage * data = NULL;
  if (size) {
    data = (jetbot_actions__action__BasicControl_FeedbackMessage *)calloc(size, sizeof(jetbot_actions__action__BasicControl_FeedbackMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jetbot_actions__action__BasicControl_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jetbot_actions__action__BasicControl_FeedbackMessage__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jetbot_actions__action__BasicControl_FeedbackMessage__Sequence__fini(jetbot_actions__action__BasicControl_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jetbot_actions__action__BasicControl_FeedbackMessage__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jetbot_actions__action__BasicControl_FeedbackMessage__Sequence *
jetbot_actions__action__BasicControl_FeedbackMessage__Sequence__create(size_t size)
{
  jetbot_actions__action__BasicControl_FeedbackMessage__Sequence * array = (jetbot_actions__action__BasicControl_FeedbackMessage__Sequence *)malloc(sizeof(jetbot_actions__action__BasicControl_FeedbackMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = jetbot_actions__action__BasicControl_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
jetbot_actions__action__BasicControl_FeedbackMessage__Sequence__destroy(jetbot_actions__action__BasicControl_FeedbackMessage__Sequence * array)
{
  if (array) {
    jetbot_actions__action__BasicControl_FeedbackMessage__Sequence__fini(array);
  }
  free(array);
}
