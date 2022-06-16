// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:srv/Task.idl
// generated code does not contain a copyright notice
#include "interfaces/srv/detail/task__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
interfaces__srv__Task_Request__init(interfaces__srv__Task_Request * msg)
{
  if (!msg) {
    return false;
  }
  // id
  return true;
}

void
interfaces__srv__Task_Request__fini(interfaces__srv__Task_Request * msg)
{
  if (!msg) {
    return;
  }
  // id
}

interfaces__srv__Task_Request *
interfaces__srv__Task_Request__create()
{
  interfaces__srv__Task_Request * msg = (interfaces__srv__Task_Request *)malloc(sizeof(interfaces__srv__Task_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__srv__Task_Request));
  bool success = interfaces__srv__Task_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
interfaces__srv__Task_Request__destroy(interfaces__srv__Task_Request * msg)
{
  if (msg) {
    interfaces__srv__Task_Request__fini(msg);
  }
  free(msg);
}


bool
interfaces__srv__Task_Request__Sequence__init(interfaces__srv__Task_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  interfaces__srv__Task_Request * data = NULL;
  if (size) {
    data = (interfaces__srv__Task_Request *)calloc(size, sizeof(interfaces__srv__Task_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__srv__Task_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__srv__Task_Request__fini(&data[i - 1]);
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
interfaces__srv__Task_Request__Sequence__fini(interfaces__srv__Task_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__srv__Task_Request__fini(&array->data[i]);
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

interfaces__srv__Task_Request__Sequence *
interfaces__srv__Task_Request__Sequence__create(size_t size)
{
  interfaces__srv__Task_Request__Sequence * array = (interfaces__srv__Task_Request__Sequence *)malloc(sizeof(interfaces__srv__Task_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = interfaces__srv__Task_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
interfaces__srv__Task_Request__Sequence__destroy(interfaces__srv__Task_Request__Sequence * array)
{
  if (array) {
    interfaces__srv__Task_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `task`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
interfaces__srv__Task_Response__init(interfaces__srv__Task_Response * msg)
{
  if (!msg) {
    return false;
  }
  // task
  if (!geometry_msgs__msg__Point__init(&msg->task)) {
    interfaces__srv__Task_Response__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__srv__Task_Response__fini(interfaces__srv__Task_Response * msg)
{
  if (!msg) {
    return;
  }
  // task
  geometry_msgs__msg__Point__fini(&msg->task);
}

interfaces__srv__Task_Response *
interfaces__srv__Task_Response__create()
{
  interfaces__srv__Task_Response * msg = (interfaces__srv__Task_Response *)malloc(sizeof(interfaces__srv__Task_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__srv__Task_Response));
  bool success = interfaces__srv__Task_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
interfaces__srv__Task_Response__destroy(interfaces__srv__Task_Response * msg)
{
  if (msg) {
    interfaces__srv__Task_Response__fini(msg);
  }
  free(msg);
}


bool
interfaces__srv__Task_Response__Sequence__init(interfaces__srv__Task_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  interfaces__srv__Task_Response * data = NULL;
  if (size) {
    data = (interfaces__srv__Task_Response *)calloc(size, sizeof(interfaces__srv__Task_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__srv__Task_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__srv__Task_Response__fini(&data[i - 1]);
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
interfaces__srv__Task_Response__Sequence__fini(interfaces__srv__Task_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__srv__Task_Response__fini(&array->data[i]);
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

interfaces__srv__Task_Response__Sequence *
interfaces__srv__Task_Response__Sequence__create(size_t size)
{
  interfaces__srv__Task_Response__Sequence * array = (interfaces__srv__Task_Response__Sequence *)malloc(sizeof(interfaces__srv__Task_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = interfaces__srv__Task_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
interfaces__srv__Task_Response__Sequence__destroy(interfaces__srv__Task_Response__Sequence * array)
{
  if (array) {
    interfaces__srv__Task_Response__Sequence__fini(array);
  }
  free(array);
}
