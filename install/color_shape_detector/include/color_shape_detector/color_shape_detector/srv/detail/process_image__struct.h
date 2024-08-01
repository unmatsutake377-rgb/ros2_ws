// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from color_shape_detector:srv/ProcessImage.idl
// generated code does not contain a copyright notice

#ifndef COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__STRUCT_H_
#define COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ProcessImage in the package color_shape_detector.
typedef struct color_shape_detector__srv__ProcessImage_Request
{
  int32_t request;
} color_shape_detector__srv__ProcessImage_Request;

// Struct for a sequence of color_shape_detector__srv__ProcessImage_Request.
typedef struct color_shape_detector__srv__ProcessImage_Request__Sequence
{
  color_shape_detector__srv__ProcessImage_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} color_shape_detector__srv__ProcessImage_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ProcessImage in the package color_shape_detector.
typedef struct color_shape_detector__srv__ProcessImage_Response
{
  int32_t response;
} color_shape_detector__srv__ProcessImage_Response;

// Struct for a sequence of color_shape_detector__srv__ProcessImage_Response.
typedef struct color_shape_detector__srv__ProcessImage_Response__Sequence
{
  color_shape_detector__srv__ProcessImage_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} color_shape_detector__srv__ProcessImage_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__STRUCT_H_
