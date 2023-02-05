// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__FUNCTIONS_H_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hexapod_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "hexapod_interfaces/msg/detail/target_angles__struct.h"

/// Initialize msg/TargetAngles message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hexapod_interfaces__msg__TargetAngles
 * )) before or use
 * hexapod_interfaces__msg__TargetAngles__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
bool
hexapod_interfaces__msg__TargetAngles__init(hexapod_interfaces__msg__TargetAngles * msg);

/// Finalize msg/TargetAngles message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
void
hexapod_interfaces__msg__TargetAngles__fini(hexapod_interfaces__msg__TargetAngles * msg);

/// Create msg/TargetAngles message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hexapod_interfaces__msg__TargetAngles__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
hexapod_interfaces__msg__TargetAngles *
hexapod_interfaces__msg__TargetAngles__create();

/// Destroy msg/TargetAngles message.
/**
 * It calls
 * hexapod_interfaces__msg__TargetAngles__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
void
hexapod_interfaces__msg__TargetAngles__destroy(hexapod_interfaces__msg__TargetAngles * msg);

/// Check for msg/TargetAngles message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
bool
hexapod_interfaces__msg__TargetAngles__are_equal(const hexapod_interfaces__msg__TargetAngles * lhs, const hexapod_interfaces__msg__TargetAngles * rhs);

/// Copy a msg/TargetAngles message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
bool
hexapod_interfaces__msg__TargetAngles__copy(
  const hexapod_interfaces__msg__TargetAngles * input,
  hexapod_interfaces__msg__TargetAngles * output);

/// Initialize array of msg/TargetAngles messages.
/**
 * It allocates the memory for the number of elements and calls
 * hexapod_interfaces__msg__TargetAngles__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
bool
hexapod_interfaces__msg__TargetAngles__Sequence__init(hexapod_interfaces__msg__TargetAngles__Sequence * array, size_t size);

/// Finalize array of msg/TargetAngles messages.
/**
 * It calls
 * hexapod_interfaces__msg__TargetAngles__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
void
hexapod_interfaces__msg__TargetAngles__Sequence__fini(hexapod_interfaces__msg__TargetAngles__Sequence * array);

/// Create array of msg/TargetAngles messages.
/**
 * It allocates the memory for the array and calls
 * hexapod_interfaces__msg__TargetAngles__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
hexapod_interfaces__msg__TargetAngles__Sequence *
hexapod_interfaces__msg__TargetAngles__Sequence__create(size_t size);

/// Destroy array of msg/TargetAngles messages.
/**
 * It calls
 * hexapod_interfaces__msg__TargetAngles__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
void
hexapod_interfaces__msg__TargetAngles__Sequence__destroy(hexapod_interfaces__msg__TargetAngles__Sequence * array);

/// Check for msg/TargetAngles message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
bool
hexapod_interfaces__msg__TargetAngles__Sequence__are_equal(const hexapod_interfaces__msg__TargetAngles__Sequence * lhs, const hexapod_interfaces__msg__TargetAngles__Sequence * rhs);

/// Copy an array of msg/TargetAngles messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hexapod_interfaces
bool
hexapod_interfaces__msg__TargetAngles__Sequence__copy(
  const hexapod_interfaces__msg__TargetAngles__Sequence * input,
  hexapod_interfaces__msg__TargetAngles__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__FUNCTIONS_H_
