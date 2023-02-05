// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__STRUCT_H_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TargetAngles in the package hexapod_interfaces.
typedef struct hexapod_interfaces__msg__TargetAngles
{
  float shoulder_angle[7];
  float hip_angle[7];
  float knee_angle[7];
} hexapod_interfaces__msg__TargetAngles;

// Struct for a sequence of hexapod_interfaces__msg__TargetAngles.
typedef struct hexapod_interfaces__msg__TargetAngles__Sequence
{
  hexapod_interfaces__msg__TargetAngles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hexapod_interfaces__msg__TargetAngles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__STRUCT_H_
