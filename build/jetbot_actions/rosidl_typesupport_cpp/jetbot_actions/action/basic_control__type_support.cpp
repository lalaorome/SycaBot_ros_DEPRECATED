// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from jetbot_actions:action/BasicControl.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "jetbot_actions/action/detail/basic_control__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_Goal_type_support_ids_t;

static const _BasicControl_Goal_type_support_ids_t _BasicControl_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_Goal_type_support_symbol_names_t _BasicControl_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_Goal)),
  }
};

typedef struct _BasicControl_Goal_type_support_data_t
{
  void * data[2];
} _BasicControl_Goal_type_support_data_t;

static _BasicControl_Goal_type_support_data_t _BasicControl_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_Goal_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_Goal>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_Goal)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_Result_type_support_ids_t;

static const _BasicControl_Result_type_support_ids_t _BasicControl_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_Result_type_support_symbol_names_t _BasicControl_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_Result)),
  }
};

typedef struct _BasicControl_Result_type_support_data_t
{
  void * data[2];
} _BasicControl_Result_type_support_data_t;

static _BasicControl_Result_type_support_data_t _BasicControl_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_Result_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_Result_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_Result_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_Result>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_Result)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_Feedback_type_support_ids_t;

static const _BasicControl_Feedback_type_support_ids_t _BasicControl_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_Feedback_type_support_symbol_names_t _BasicControl_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_Feedback)),
  }
};

typedef struct _BasicControl_Feedback_type_support_data_t
{
  void * data[2];
} _BasicControl_Feedback_type_support_data_t;

static _BasicControl_Feedback_type_support_data_t _BasicControl_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_Feedback_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_Feedback>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_Feedback)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_SendGoal_Request_type_support_ids_t;

static const _BasicControl_SendGoal_Request_type_support_ids_t _BasicControl_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_SendGoal_Request_type_support_symbol_names_t _BasicControl_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_SendGoal_Request)),
  }
};

typedef struct _BasicControl_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _BasicControl_SendGoal_Request_type_support_data_t;

static _BasicControl_SendGoal_Request_type_support_data_t _BasicControl_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_SendGoal_Request_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_SendGoal_Request>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_SendGoal_Request)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_SendGoal_Response_type_support_ids_t;

static const _BasicControl_SendGoal_Response_type_support_ids_t _BasicControl_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_SendGoal_Response_type_support_symbol_names_t _BasicControl_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_SendGoal_Response)),
  }
};

typedef struct _BasicControl_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _BasicControl_SendGoal_Response_type_support_data_t;

static _BasicControl_SendGoal_Response_type_support_data_t _BasicControl_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_SendGoal_Response_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_SendGoal_Response>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_SendGoal_Response)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_SendGoal_type_support_ids_t;

static const _BasicControl_SendGoal_type_support_ids_t _BasicControl_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_SendGoal_type_support_symbol_names_t _BasicControl_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_SendGoal)),
  }
};

typedef struct _BasicControl_SendGoal_type_support_data_t
{
  void * data[2];
} _BasicControl_SendGoal_type_support_data_t;

static _BasicControl_SendGoal_type_support_data_t _BasicControl_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_SendGoal_service_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t BasicControl_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<jetbot_actions::action::BasicControl_SendGoal>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_GetResult_Request_type_support_ids_t;

static const _BasicControl_GetResult_Request_type_support_ids_t _BasicControl_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_GetResult_Request_type_support_symbol_names_t _BasicControl_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_GetResult_Request)),
  }
};

typedef struct _BasicControl_GetResult_Request_type_support_data_t
{
  void * data[2];
} _BasicControl_GetResult_Request_type_support_data_t;

static _BasicControl_GetResult_Request_type_support_data_t _BasicControl_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_GetResult_Request_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_GetResult_Request>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_GetResult_Request)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_GetResult_Response_type_support_ids_t;

static const _BasicControl_GetResult_Response_type_support_ids_t _BasicControl_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_GetResult_Response_type_support_symbol_names_t _BasicControl_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_GetResult_Response)),
  }
};

typedef struct _BasicControl_GetResult_Response_type_support_data_t
{
  void * data[2];
} _BasicControl_GetResult_Response_type_support_data_t;

static _BasicControl_GetResult_Response_type_support_data_t _BasicControl_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_GetResult_Response_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_GetResult_Response>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_GetResult_Response)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_GetResult_type_support_ids_t;

static const _BasicControl_GetResult_type_support_ids_t _BasicControl_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_GetResult_type_support_symbol_names_t _BasicControl_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_GetResult)),
  }
};

typedef struct _BasicControl_GetResult_type_support_data_t
{
  void * data[2];
} _BasicControl_GetResult_type_support_data_t;

static _BasicControl_GetResult_type_support_data_t _BasicControl_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_GetResult_service_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t BasicControl_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<jetbot_actions::action::BasicControl_GetResult>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _BasicControl_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _BasicControl_FeedbackMessage_type_support_ids_t;

static const _BasicControl_FeedbackMessage_type_support_ids_t _BasicControl_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _BasicControl_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _BasicControl_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _BasicControl_FeedbackMessage_type_support_symbol_names_t _BasicControl_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jetbot_actions, action, BasicControl_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jetbot_actions, action, BasicControl_FeedbackMessage)),
  }
};

typedef struct _BasicControl_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _BasicControl_FeedbackMessage_type_support_data_t;

static _BasicControl_FeedbackMessage_type_support_data_t _BasicControl_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _BasicControl_FeedbackMessage_message_typesupport_map = {
  2,
  "jetbot_actions",
  &_BasicControl_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_BasicControl_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_BasicControl_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t BasicControl_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_BasicControl_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jetbot_actions::action::BasicControl_FeedbackMessage>()
{
  return &::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, jetbot_actions, action, BasicControl_FeedbackMessage)() {
  return get_message_type_support_handle<jetbot_actions::action::BasicControl_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "jetbot_actions/action/detail/basic_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace jetbot_actions
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t BasicControl_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace jetbot_actions

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<jetbot_actions::action::BasicControl>()
{
  using ::jetbot_actions::action::rosidl_typesupport_cpp::BasicControl_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  BasicControl_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::jetbot_actions::action::BasicControl::Impl::SendGoalService>();
  BasicControl_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::jetbot_actions::action::BasicControl::Impl::GetResultService>();
  BasicControl_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::jetbot_actions::action::BasicControl::Impl::CancelGoalService>();
  BasicControl_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::jetbot_actions::action::BasicControl::Impl::FeedbackMessage>();
  BasicControl_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::jetbot_actions::action::BasicControl::Impl::GoalStatusMessage>();
  return &BasicControl_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
