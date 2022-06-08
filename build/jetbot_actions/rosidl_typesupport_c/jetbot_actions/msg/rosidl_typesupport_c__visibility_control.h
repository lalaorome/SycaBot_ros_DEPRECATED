// generated from
// rosidl_typesupport_c/resource/rosidl_typesupport_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef JETBOT_ACTIONS__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_
#define JETBOT_ACTIONS__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_C_EXPORT_jetbot_actions __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_C_IMPORT_jetbot_actions __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_C_EXPORT_jetbot_actions __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_C_IMPORT_jetbot_actions __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_C_BUILDING_DLL_jetbot_actions
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_jetbot_actions ROSIDL_TYPESUPPORT_C_EXPORT_jetbot_actions
  #else
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_jetbot_actions ROSIDL_TYPESUPPORT_C_IMPORT_jetbot_actions
  #endif
#else
  #define ROSIDL_TYPESUPPORT_C_EXPORT_jetbot_actions __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_C_IMPORT_jetbot_actions
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_jetbot_actions __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_jetbot_actions
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // JETBOT_ACTIONS__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_
