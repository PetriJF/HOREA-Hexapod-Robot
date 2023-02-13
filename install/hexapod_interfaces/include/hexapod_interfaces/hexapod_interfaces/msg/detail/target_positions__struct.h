// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__STRUCT_H_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TargetPositions in the package hexapod_interfaces.
typedef struct hexapod_interfaces__msg__TargetPositions
{
  float x_pos[7];
  float y_pos[7];
  float z_pos[7];
} hexapod_interfaces__msg__TargetPositions;

// Struct for a sequence of hexapod_interfaces__msg__TargetPositions.
typedef struct hexapod_interfaces__msg__TargetPositions__Sequence
{
  hexapod_interfaces__msg__TargetPositions * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hexapod_interfaces__msg__TargetPositions__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__STRUCT_H_
