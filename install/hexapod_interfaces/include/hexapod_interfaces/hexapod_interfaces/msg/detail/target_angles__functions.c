// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice
#include "hexapod_interfaces/msg/detail/target_angles__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hexapod_interfaces__msg__TargetAngles__init(hexapod_interfaces__msg__TargetAngles * msg)
{
  if (!msg) {
    return false;
  }
  // shoulder_angle
  msg->shoulder_angle[0] = 0.0f;
  msg->shoulder_angle[1] = 0.0f;
  msg->shoulder_angle[2] = 0.0f;
  msg->shoulder_angle[3] = 0.0f;
  msg->shoulder_angle[4] = 0.0f;
  msg->shoulder_angle[5] = 0.0f;
  msg->shoulder_angle[6] = 0.0f;
  // hip_angle
  msg->hip_angle[0] = 0.0f;
  msg->hip_angle[1] = 0.0f;
  msg->hip_angle[2] = 0.0f;
  msg->hip_angle[3] = 0.0f;
  msg->hip_angle[4] = 0.0f;
  msg->hip_angle[5] = 0.0f;
  msg->hip_angle[6] = 0.0f;
  // knee_angle
  msg->knee_angle[0] = 0.0f;
  msg->knee_angle[1] = 0.0f;
  msg->knee_angle[2] = 0.0f;
  msg->knee_angle[3] = 0.0f;
  msg->knee_angle[4] = 0.0f;
  msg->knee_angle[5] = 0.0f;
  msg->knee_angle[6] = 0.0f;
  return true;
}

void
hexapod_interfaces__msg__TargetAngles__fini(hexapod_interfaces__msg__TargetAngles * msg)
{
  if (!msg) {
    return;
  }
  // shoulder_angle
  // hip_angle
  // knee_angle
}

bool
hexapod_interfaces__msg__TargetAngles__are_equal(const hexapod_interfaces__msg__TargetAngles * lhs, const hexapod_interfaces__msg__TargetAngles * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // shoulder_angle
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->shoulder_angle[i] != rhs->shoulder_angle[i]) {
      return false;
    }
  }
  // hip_angle
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->hip_angle[i] != rhs->hip_angle[i]) {
      return false;
    }
  }
  // knee_angle
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->knee_angle[i] != rhs->knee_angle[i]) {
      return false;
    }
  }
  return true;
}

bool
hexapod_interfaces__msg__TargetAngles__copy(
  const hexapod_interfaces__msg__TargetAngles * input,
  hexapod_interfaces__msg__TargetAngles * output)
{
  if (!input || !output) {
    return false;
  }
  // shoulder_angle
  for (size_t i = 0; i < 7; ++i) {
    output->shoulder_angle[i] = input->shoulder_angle[i];
  }
  // hip_angle
  for (size_t i = 0; i < 7; ++i) {
    output->hip_angle[i] = input->hip_angle[i];
  }
  // knee_angle
  for (size_t i = 0; i < 7; ++i) {
    output->knee_angle[i] = input->knee_angle[i];
  }
  return true;
}

hexapod_interfaces__msg__TargetAngles *
hexapod_interfaces__msg__TargetAngles__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__TargetAngles * msg = (hexapod_interfaces__msg__TargetAngles *)allocator.allocate(sizeof(hexapod_interfaces__msg__TargetAngles), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hexapod_interfaces__msg__TargetAngles));
  bool success = hexapod_interfaces__msg__TargetAngles__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hexapod_interfaces__msg__TargetAngles__destroy(hexapod_interfaces__msg__TargetAngles * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hexapod_interfaces__msg__TargetAngles__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hexapod_interfaces__msg__TargetAngles__Sequence__init(hexapod_interfaces__msg__TargetAngles__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__TargetAngles * data = NULL;

  if (size) {
    data = (hexapod_interfaces__msg__TargetAngles *)allocator.zero_allocate(size, sizeof(hexapod_interfaces__msg__TargetAngles), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hexapod_interfaces__msg__TargetAngles__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hexapod_interfaces__msg__TargetAngles__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hexapod_interfaces__msg__TargetAngles__Sequence__fini(hexapod_interfaces__msg__TargetAngles__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hexapod_interfaces__msg__TargetAngles__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hexapod_interfaces__msg__TargetAngles__Sequence *
hexapod_interfaces__msg__TargetAngles__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__TargetAngles__Sequence * array = (hexapod_interfaces__msg__TargetAngles__Sequence *)allocator.allocate(sizeof(hexapod_interfaces__msg__TargetAngles__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hexapod_interfaces__msg__TargetAngles__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hexapod_interfaces__msg__TargetAngles__Sequence__destroy(hexapod_interfaces__msg__TargetAngles__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hexapod_interfaces__msg__TargetAngles__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hexapod_interfaces__msg__TargetAngles__Sequence__are_equal(const hexapod_interfaces__msg__TargetAngles__Sequence * lhs, const hexapod_interfaces__msg__TargetAngles__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hexapod_interfaces__msg__TargetAngles__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hexapod_interfaces__msg__TargetAngles__Sequence__copy(
  const hexapod_interfaces__msg__TargetAngles__Sequence * input,
  hexapod_interfaces__msg__TargetAngles__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hexapod_interfaces__msg__TargetAngles);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hexapod_interfaces__msg__TargetAngles * data =
      (hexapod_interfaces__msg__TargetAngles *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hexapod_interfaces__msg__TargetAngles__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hexapod_interfaces__msg__TargetAngles__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hexapod_interfaces__msg__TargetAngles__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
