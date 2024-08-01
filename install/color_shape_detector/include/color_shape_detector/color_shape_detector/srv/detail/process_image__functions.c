// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from color_shape_detector:srv/ProcessImage.idl
// generated code does not contain a copyright notice
#include "color_shape_detector/srv/detail/process_image__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
color_shape_detector__srv__ProcessImage_Request__init(color_shape_detector__srv__ProcessImage_Request * msg)
{
  if (!msg) {
    return false;
  }
  // request
  return true;
}

void
color_shape_detector__srv__ProcessImage_Request__fini(color_shape_detector__srv__ProcessImage_Request * msg)
{
  if (!msg) {
    return;
  }
  // request
}

bool
color_shape_detector__srv__ProcessImage_Request__are_equal(const color_shape_detector__srv__ProcessImage_Request * lhs, const color_shape_detector__srv__ProcessImage_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request
  if (lhs->request != rhs->request) {
    return false;
  }
  return true;
}

bool
color_shape_detector__srv__ProcessImage_Request__copy(
  const color_shape_detector__srv__ProcessImage_Request * input,
  color_shape_detector__srv__ProcessImage_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // request
  output->request = input->request;
  return true;
}

color_shape_detector__srv__ProcessImage_Request *
color_shape_detector__srv__ProcessImage_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  color_shape_detector__srv__ProcessImage_Request * msg = (color_shape_detector__srv__ProcessImage_Request *)allocator.allocate(sizeof(color_shape_detector__srv__ProcessImage_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(color_shape_detector__srv__ProcessImage_Request));
  bool success = color_shape_detector__srv__ProcessImage_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
color_shape_detector__srv__ProcessImage_Request__destroy(color_shape_detector__srv__ProcessImage_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    color_shape_detector__srv__ProcessImage_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
color_shape_detector__srv__ProcessImage_Request__Sequence__init(color_shape_detector__srv__ProcessImage_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  color_shape_detector__srv__ProcessImage_Request * data = NULL;

  if (size) {
    data = (color_shape_detector__srv__ProcessImage_Request *)allocator.zero_allocate(size, sizeof(color_shape_detector__srv__ProcessImage_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = color_shape_detector__srv__ProcessImage_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        color_shape_detector__srv__ProcessImage_Request__fini(&data[i - 1]);
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
color_shape_detector__srv__ProcessImage_Request__Sequence__fini(color_shape_detector__srv__ProcessImage_Request__Sequence * array)
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
      color_shape_detector__srv__ProcessImage_Request__fini(&array->data[i]);
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

color_shape_detector__srv__ProcessImage_Request__Sequence *
color_shape_detector__srv__ProcessImage_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  color_shape_detector__srv__ProcessImage_Request__Sequence * array = (color_shape_detector__srv__ProcessImage_Request__Sequence *)allocator.allocate(sizeof(color_shape_detector__srv__ProcessImage_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = color_shape_detector__srv__ProcessImage_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
color_shape_detector__srv__ProcessImage_Request__Sequence__destroy(color_shape_detector__srv__ProcessImage_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    color_shape_detector__srv__ProcessImage_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
color_shape_detector__srv__ProcessImage_Request__Sequence__are_equal(const color_shape_detector__srv__ProcessImage_Request__Sequence * lhs, const color_shape_detector__srv__ProcessImage_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!color_shape_detector__srv__ProcessImage_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
color_shape_detector__srv__ProcessImage_Request__Sequence__copy(
  const color_shape_detector__srv__ProcessImage_Request__Sequence * input,
  color_shape_detector__srv__ProcessImage_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(color_shape_detector__srv__ProcessImage_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    color_shape_detector__srv__ProcessImage_Request * data =
      (color_shape_detector__srv__ProcessImage_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!color_shape_detector__srv__ProcessImage_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          color_shape_detector__srv__ProcessImage_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!color_shape_detector__srv__ProcessImage_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
color_shape_detector__srv__ProcessImage_Response__init(color_shape_detector__srv__ProcessImage_Response * msg)
{
  if (!msg) {
    return false;
  }
  // response
  return true;
}

void
color_shape_detector__srv__ProcessImage_Response__fini(color_shape_detector__srv__ProcessImage_Response * msg)
{
  if (!msg) {
    return;
  }
  // response
}

bool
color_shape_detector__srv__ProcessImage_Response__are_equal(const color_shape_detector__srv__ProcessImage_Response * lhs, const color_shape_detector__srv__ProcessImage_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // response
  if (lhs->response != rhs->response) {
    return false;
  }
  return true;
}

bool
color_shape_detector__srv__ProcessImage_Response__copy(
  const color_shape_detector__srv__ProcessImage_Response * input,
  color_shape_detector__srv__ProcessImage_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // response
  output->response = input->response;
  return true;
}

color_shape_detector__srv__ProcessImage_Response *
color_shape_detector__srv__ProcessImage_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  color_shape_detector__srv__ProcessImage_Response * msg = (color_shape_detector__srv__ProcessImage_Response *)allocator.allocate(sizeof(color_shape_detector__srv__ProcessImage_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(color_shape_detector__srv__ProcessImage_Response));
  bool success = color_shape_detector__srv__ProcessImage_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
color_shape_detector__srv__ProcessImage_Response__destroy(color_shape_detector__srv__ProcessImage_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    color_shape_detector__srv__ProcessImage_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
color_shape_detector__srv__ProcessImage_Response__Sequence__init(color_shape_detector__srv__ProcessImage_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  color_shape_detector__srv__ProcessImage_Response * data = NULL;

  if (size) {
    data = (color_shape_detector__srv__ProcessImage_Response *)allocator.zero_allocate(size, sizeof(color_shape_detector__srv__ProcessImage_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = color_shape_detector__srv__ProcessImage_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        color_shape_detector__srv__ProcessImage_Response__fini(&data[i - 1]);
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
color_shape_detector__srv__ProcessImage_Response__Sequence__fini(color_shape_detector__srv__ProcessImage_Response__Sequence * array)
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
      color_shape_detector__srv__ProcessImage_Response__fini(&array->data[i]);
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

color_shape_detector__srv__ProcessImage_Response__Sequence *
color_shape_detector__srv__ProcessImage_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  color_shape_detector__srv__ProcessImage_Response__Sequence * array = (color_shape_detector__srv__ProcessImage_Response__Sequence *)allocator.allocate(sizeof(color_shape_detector__srv__ProcessImage_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = color_shape_detector__srv__ProcessImage_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
color_shape_detector__srv__ProcessImage_Response__Sequence__destroy(color_shape_detector__srv__ProcessImage_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    color_shape_detector__srv__ProcessImage_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
color_shape_detector__srv__ProcessImage_Response__Sequence__are_equal(const color_shape_detector__srv__ProcessImage_Response__Sequence * lhs, const color_shape_detector__srv__ProcessImage_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!color_shape_detector__srv__ProcessImage_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
color_shape_detector__srv__ProcessImage_Response__Sequence__copy(
  const color_shape_detector__srv__ProcessImage_Response__Sequence * input,
  color_shape_detector__srv__ProcessImage_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(color_shape_detector__srv__ProcessImage_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    color_shape_detector__srv__ProcessImage_Response * data =
      (color_shape_detector__srv__ProcessImage_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!color_shape_detector__srv__ProcessImage_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          color_shape_detector__srv__ProcessImage_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!color_shape_detector__srv__ProcessImage_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
