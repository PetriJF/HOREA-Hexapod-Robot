// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "hexapod_interfaces/msg/detail/target_angles__struct.h"
#include "hexapod_interfaces/msg/detail/target_angles__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace hexapod_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _TargetAngles_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TargetAngles_type_support_ids_t;

static const _TargetAngles_type_support_ids_t _TargetAngles_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TargetAngles_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TargetAngles_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TargetAngles_type_support_symbol_names_t _TargetAngles_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hexapod_interfaces, msg, TargetAngles)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hexapod_interfaces, msg, TargetAngles)),
  }
};

typedef struct _TargetAngles_type_support_data_t
{
  void * data[2];
} _TargetAngles_type_support_data_t;

static _TargetAngles_type_support_data_t _TargetAngles_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TargetAngles_message_typesupport_map = {
  2,
  "hexapod_interfaces",
  &_TargetAngles_message_typesupport_ids.typesupport_identifier[0],
  &_TargetAngles_message_typesupport_symbol_names.symbol_name[0],
  &_TargetAngles_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TargetAngles_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TargetAngles_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace hexapod_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, hexapod_interfaces, msg, TargetAngles)() {
  return &::hexapod_interfaces::msg::rosidl_typesupport_c::TargetAngles_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
