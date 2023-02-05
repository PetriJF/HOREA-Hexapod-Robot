// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hexapod_interfaces/msg/detail/target_angles__struct.hpp"
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

void TargetAngles_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hexapod_interfaces::msg::TargetAngles(_init);
}

void TargetAngles_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hexapod_interfaces::msg::TargetAngles *>(message_memory);
  typed_message->~TargetAngles();
}

size_t size_function__TargetAngles__shoulder_angle(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__TargetAngles__shoulder_angle(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetAngles__shoulder_angle(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetAngles__shoulder_angle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TargetAngles__shoulder_angle(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TargetAngles__shoulder_angle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TargetAngles__shoulder_angle(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__TargetAngles__hip_angle(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__TargetAngles__hip_angle(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetAngles__hip_angle(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetAngles__hip_angle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TargetAngles__hip_angle(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TargetAngles__hip_angle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TargetAngles__hip_angle(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__TargetAngles__knee_angle(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__TargetAngles__knee_angle(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__TargetAngles__knee_angle(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__TargetAngles__knee_angle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TargetAngles__knee_angle(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TargetAngles__knee_angle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TargetAngles__knee_angle(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TargetAngles_message_member_array[3] = {
  {
    "shoulder_angle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(hexapod_interfaces::msg::TargetAngles, shoulder_angle),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetAngles__shoulder_angle,  // size() function pointer
    get_const_function__TargetAngles__shoulder_angle,  // get_const(index) function pointer
    get_function__TargetAngles__shoulder_angle,  // get(index) function pointer
    fetch_function__TargetAngles__shoulder_angle,  // fetch(index, &value) function pointer
    assign_function__TargetAngles__shoulder_angle,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "hip_angle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(hexapod_interfaces::msg::TargetAngles, hip_angle),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetAngles__hip_angle,  // size() function pointer
    get_const_function__TargetAngles__hip_angle,  // get_const(index) function pointer
    get_function__TargetAngles__hip_angle,  // get(index) function pointer
    fetch_function__TargetAngles__hip_angle,  // fetch(index, &value) function pointer
    assign_function__TargetAngles__hip_angle,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "knee_angle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(hexapod_interfaces::msg::TargetAngles, knee_angle),  // bytes offset in struct
    nullptr,  // default value
    size_function__TargetAngles__knee_angle,  // size() function pointer
    get_const_function__TargetAngles__knee_angle,  // get_const(index) function pointer
    get_function__TargetAngles__knee_angle,  // get(index) function pointer
    fetch_function__TargetAngles__knee_angle,  // fetch(index, &value) function pointer
    assign_function__TargetAngles__knee_angle,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TargetAngles_message_members = {
  "hexapod_interfaces::msg",  // message namespace
  "TargetAngles",  // message name
  3,  // number of fields
  sizeof(hexapod_interfaces::msg::TargetAngles),
  TargetAngles_message_member_array,  // message members
  TargetAngles_init_function,  // function to initialize message memory (memory has to be allocated)
  TargetAngles_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TargetAngles_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TargetAngles_message_members,
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
get_message_type_support_handle<hexapod_interfaces::msg::TargetAngles>()
{
  return &::hexapod_interfaces::msg::rosidl_typesupport_introspection_cpp::TargetAngles_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hexapod_interfaces, msg, TargetAngles)() {
  return &::hexapod_interfaces::msg::rosidl_typesupport_introspection_cpp::TargetAngles_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
