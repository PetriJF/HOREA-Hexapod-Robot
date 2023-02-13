// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice
#include "hexapod_interfaces/msg/detail/target_positions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hexapod_interfaces__msg__TargetPositions__init(hexapod_interfaces__msg__TargetPositions * msg)
{
  if (!msg) {
    return false;
  }
  // x_pos
  msg->x_pos[0] = 0.0f;
  msg->x_pos[1] = 0.0f;
  msg->x_pos[2] = 0.0f;
  msg->x_pos[3] = 0.0f;
  msg->x_pos[4] = 0.0f;
  msg->x_pos[5] = 0.0f;
  msg->x_pos[6] = 0.0f;
  // y_pos
  msg->y_pos[0] = 0.0f;
  msg->y_pos[1] = 0.0f;
  msg->y_pos[2] = 0.0f;
  msg->y_pos[3] = 0.0f;
  msg->y_pos[4] = 0.0f;
  msg->y_pos[5] = 0.0f;
  msg->y_pos[6] = 0.0f;
  // z_pos
  msg->z_pos[0] = 0.0f;
  msg->z_pos[1] = 0.0f;
  msg->z_pos[2] = 0.0f;
  msg->z_pos[3] = 0.0f;
  msg->z_pos[4] = 0.0f;
  msg->z_pos[5] = 0.0f;
  msg->z_pos[6] = 0.0f;
  return true;
}

void
hexapod_interfaces__msg__TargetPositions__fini(hexapod_interfaces__msg__TargetPositions * msg)
{
  if (!msg) {
    return;
  }
  // x_pos
  // y_pos
  // z_pos
}

bool
hexapod_interfaces__msg__TargetPositions__are_equal(const hexapod_interfaces__msg__TargetPositions * lhs, const hexapod_interfaces__msg__TargetPositions * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x_pos
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->x_pos[i] != rhs->x_pos[i]) {
      return false;
    }
  }
  // y_pos
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->y_pos[i] != rhs->y_pos[i]) {
      return false;
    }
  }
  // z_pos
  for (size_t i = 0; i < 7; ++i) {
    if (lhs->z_pos[i] != rhs->z_pos[i]) {
      return false;
    }
  }
  return true;
}

bool
hexapod_interfaces__msg__TargetPositions__copy(
  const hexapod_interfaces__msg__TargetPositions * input,
  hexapod_interfaces__msg__TargetPositions * output)
{
  if (!input || !output) {
    return false;
  }
  // x_pos
  for (size_t i = 0; i < 7; ++i) {
    output->x_pos[i] = input->x_pos[i];
  }
  // y_pos
  for (size_t i = 0; i < 7; ++i) {
    output->y_pos[i] = input->y_pos[i];
  }
  // z_pos
  for (size_t i = 0; i < 7; ++i) {
    output->z_pos[i] = input->z_pos[i];
  }
  return true;
}

hexapod_interfaces__msg__TargetPositions *
hexapod_interfaces__msg__TargetPositions__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__TargetPositions * msg = (hexapod_interfaces__msg__TargetPositions *)allocator.allocate(sizeof(hexapod_interfaces__msg__TargetPositions), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hexapod_interfaces__msg__TargetPositions));
  bool success = hexapod_interfaces__msg__TargetPositions__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hexapod_interfaces__msg__TargetPositions__destroy(hexapod_interfaces__msg__TargetPositions * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hexapod_interfaces__msg__TargetPositions__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hexapod_interfaces__msg__TargetPositions__Sequence__init(hexapod_interfaces__msg__TargetPositions__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__TargetPositions * data = NULL;

  if (size) {
    data = (hexapod_interfaces__msg__TargetPositions *)allocator.zero_allocate(size, sizeof(hexapod_interfaces__msg__TargetPositions), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hexapod_interfaces__msg__TargetPositions__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hexapod_interfaces__msg__TargetPositions__fini(&data[i - 1]);
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
hexapod_interfaces__msg__TargetPositions__Sequence__fini(hexapod_interfaces__msg__TargetPositions__Sequence * array)
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
      hexapod_interfaces__msg__TargetPositions__fini(&array->data[i]);
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

hexapod_interfaces__msg__TargetPositions__Sequence *
hexapod_interfaces__msg__TargetPositions__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hexapod_interfaces__msg__TargetPositions__Sequence * array = (hexapod_interfaces__msg__TargetPositions__Sequence *)allocator.allocate(sizeof(hexapod_interfaces__msg__TargetPositions__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hexapod_interfaces__msg__TargetPositions__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hexapod_interfaces__msg__TargetPositions__Sequence__destroy(hexapod_interfaces__msg__TargetPositions__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hexapod_interfaces__msg__TargetPositions__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hexapod_interfaces__msg__TargetPositions__Sequence__are_equal(const hexapod_interfaces__msg__TargetPositions__Sequence * lhs, const hexapod_interfaces__msg__TargetPositions__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hexapod_interfaces__msg__TargetPositions__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hexapod_interfaces__msg__TargetPositions__Sequence__copy(
  const hexapod_interfaces__msg__TargetPositions__Sequence * input,
  hexapod_interfaces__msg__TargetPositions__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hexapod_interfaces__msg__TargetPositions);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hexapod_interfaces__msg__TargetPositions * data =
      (hexapod_interfaces__msg__TargetPositions *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hexapod_interfaces__msg__TargetPositions__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hexapod_interfaces__msg__TargetPositions__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hexapod_interfaces__msg__TargetPositions__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
