// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice
#include "hexapod_interfaces/msg/detail/target_positions__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "hexapod_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hexapod_interfaces/msg/detail/target_positions__struct.h"
#include "hexapod_interfaces/msg/detail/target_positions__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TargetPositions__ros_msg_type = hexapod_interfaces__msg__TargetPositions;

static bool _TargetPositions__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TargetPositions__ros_msg_type * ros_message = static_cast<const _TargetPositions__ros_msg_type *>(untyped_ros_message);
  // Field name: x_pos
  {
    size_t size = 7;
    auto array_ptr = ros_message->x_pos;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: y_pos
  {
    size_t size = 7;
    auto array_ptr = ros_message->y_pos;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: z_pos
  {
    size_t size = 7;
    auto array_ptr = ros_message->z_pos;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _TargetPositions__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TargetPositions__ros_msg_type * ros_message = static_cast<_TargetPositions__ros_msg_type *>(untyped_ros_message);
  // Field name: x_pos
  {
    size_t size = 7;
    auto array_ptr = ros_message->x_pos;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: y_pos
  {
    size_t size = 7;
    auto array_ptr = ros_message->y_pos;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: z_pos
  {
    size_t size = 7;
    auto array_ptr = ros_message->z_pos;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hexapod_interfaces
size_t get_serialized_size_hexapod_interfaces__msg__TargetPositions(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TargetPositions__ros_msg_type * ros_message = static_cast<const _TargetPositions__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name x_pos
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->x_pos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y_pos
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->y_pos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z_pos
  {
    size_t array_size = 7;
    auto array_ptr = ros_message->z_pos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TargetPositions__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_hexapod_interfaces__msg__TargetPositions(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hexapod_interfaces
size_t max_serialized_size_hexapod_interfaces__msg__TargetPositions(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: x_pos
  {
    size_t array_size = 7;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: y_pos
  {
    size_t array_size = 7;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: z_pos
  {
    size_t array_size = 7;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _TargetPositions__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_hexapod_interfaces__msg__TargetPositions(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TargetPositions = {
  "hexapod_interfaces::msg",
  "TargetPositions",
  _TargetPositions__cdr_serialize,
  _TargetPositions__cdr_deserialize,
  _TargetPositions__get_serialized_size,
  _TargetPositions__max_serialized_size
};

static rosidl_message_type_support_t _TargetPositions__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TargetPositions,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hexapod_interfaces, msg, TargetPositions)() {
  return &_TargetPositions__type_support;
}

#if defined(__cplusplus)
}
#endif
