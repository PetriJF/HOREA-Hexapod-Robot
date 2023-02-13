// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hexapod_interfaces/msg/detail/target_positions__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hexapod_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TargetPositions_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hexapod_interfaces::msg::TargetPositions(_init);
}

void TargetPositions_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hexapod_interfaces::msg::TargetPositions *>(message_memory);
  typed_message->~TargetPositions();
}

size_t size_function__TargetPositions__x_pos(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__TargetPositions__x_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetPositions__x_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetPositions__x_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TargetPositions__x_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TargetPositions__x_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TargetPositions__x_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__TargetPositions__y_pos(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__TargetPositions__y_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetPositions__y_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetPositions__y_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TargetPositions__y_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TargetPositions__y_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TargetPositions__y_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__TargetPositions__z_pos(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__TargetPositions__z_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetPositions__z_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetPositions__z_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TargetPositions__z_pos(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TargetPositions__z_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TargetPositions__z_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TargetPositions_message_member_array[3] = {
  {
    "x_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(hexapod_interfaces::msg::TargetPositions, x_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetPositions__x_pos,  // size() function pointer
    get_const_function__TargetPositions__x_pos,  // get_const(index) function pointer
    get_function__TargetPositions__x_pos,  // get(index) function pointer
    fetch_function__TargetPositions__x_pos,  // fetch(index, &value) function pointer
    assign_function__TargetPositions__x_pos,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "y_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(hexapod_interfaces::msg::TargetPositions, y_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetPositions__y_pos,  // size() function pointer
    get_const_function__TargetPositions__y_pos,  // get_const(index) function pointer
    get_function__TargetPositions__y_pos,  // get(index) function pointer
    fetch_function__TargetPositions__y_pos,  // fetch(index, &value) function pointer
    assign_function__TargetPositions__y_pos,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "z_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(hexapod_interfaces::msg::TargetPositions, z_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetPositions__z_pos,  // size() function pointer
    get_const_function__TargetPositions__z_pos,  // get_const(index) function pointer
    get_function__TargetPositions__z_pos,  // get(index) function pointer
    fetch_function__TargetPositions__z_pos,  // fetch(index, &value) function pointer
    assign_function__TargetPositions__z_pos,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TargetPositions_message_members = {
  "hexapod_interfaces::msg",  // message namespace
  "TargetPositions",  // message name
  3,  // number of fields
  sizeof(hexapod_interfaces::msg::TargetPositions),
  TargetPositions_message_member_array,  // message members
  TargetPositions_init_function,  // function to initialize message memory (memory has to be allocated)
  TargetPositions_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TargetPositions_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TargetPositions_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace hexapod_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hexapod_interfaces::msg::TargetPositions>()
{
  return &::hexapod_interfaces::msg::rosidl_typesupport_introspection_cpp::TargetPositions_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hexapod_interfaces, msg, TargetPositions)() {
  return &::hexapod_interfaces::msg::rosidl_typesupport_introspection_cpp::TargetPositions_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
