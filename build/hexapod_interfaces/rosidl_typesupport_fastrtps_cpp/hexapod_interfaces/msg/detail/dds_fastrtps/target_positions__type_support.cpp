// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice
#include "hexapod_interfaces/msg/detail/target_positions__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hexapod_interfaces/msg/detail/target_positions__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hexapod_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hexapod_interfaces
cdr_serialize(
  const hexapod_interfaces::msg::TargetPositions & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: x_pos
  {
    cdr << ros_message.x_pos;
  }
  // Member: y_pos
  {
    cdr << ros_message.y_pos;
  }
  // Member: z_pos
  {
    cdr << ros_message.z_pos;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hexapod_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hexapod_interfaces::msg::TargetPositions & ros_message)
{
  // Member: x_pos
  {
    cdr >> ros_message.x_pos;
  }

  // Member: y_pos
  {
    cdr >> ros_message.y_pos;
  }

  // Member: z_pos
  {
    cdr >> ros_message.z_pos;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hexapod_interfaces
get_serialized_size(
  const hexapod_interfaces::msg::TargetPositions & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: x_pos
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.x_pos[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_pos
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.y_pos[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: z_pos
  {
    size_t array_size = 7;
    size_t item_size = sizeof(ros_message.z_pos[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hexapod_interfaces
max_serialized_size_TargetPositions(
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


  // Member: x_pos
  {
    size_t array_size = 7;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: y_pos
  {
    size_t array_size = 7;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: z_pos
  {
    size_t array_size = 7;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _TargetPositions__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hexapod_interfaces::msg::TargetPositions *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TargetPositions__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hexapod_interfaces::msg::TargetPositions *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TargetPositions__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hexapod_interfaces::msg::TargetPositions *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TargetPositions__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TargetPositions(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TargetPositions__callbacks = {
  "hexapod_interfaces::msg",
  "TargetPositions",
  _TargetPositions__cdr_serialize,
  _TargetPositions__cdr_deserialize,
  _TargetPositions__get_serialized_size,
  _TargetPositions__max_serialized_size
};

static rosidl_message_type_support_t _TargetPositions__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TargetPositions__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hexapod_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hexapod_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<hexapod_interfaces::msg::TargetPositions>()
{
  return &hexapod_interfaces::msg::typesupport_fastrtps_cpp::_TargetPositions__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hexapod_interfaces, msg, TargetPositions)() {
  return &hexapod_interfaces::msg::typesupport_fastrtps_cpp::_TargetPositions__handle;
}

#ifdef __cplusplus
}
#endif
