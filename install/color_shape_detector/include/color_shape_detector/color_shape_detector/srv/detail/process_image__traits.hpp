// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from color_shape_detector:srv/ProcessImage.idl
// generated code does not contain a copyright notice

#ifndef COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__TRAITS_HPP_
#define COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "color_shape_detector/srv/detail/process_image__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace color_shape_detector
{

namespace srv
{

inline void to_flow_style_yaml(
  const ProcessImage_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ProcessImage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ProcessImage_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace color_shape_detector

namespace rosidl_generator_traits
{

[[deprecated("use color_shape_detector::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const color_shape_detector::srv::ProcessImage_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  color_shape_detector::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use color_shape_detector::srv::to_yaml() instead")]]
inline std::string to_yaml(const color_shape_detector::srv::ProcessImage_Request & msg)
{
  return color_shape_detector::srv::to_yaml(msg);
}

template<>
inline const char * data_type<color_shape_detector::srv::ProcessImage_Request>()
{
  return "color_shape_detector::srv::ProcessImage_Request";
}

template<>
inline const char * name<color_shape_detector::srv::ProcessImage_Request>()
{
  return "color_shape_detector/srv/ProcessImage_Request";
}

template<>
struct has_fixed_size<color_shape_detector::srv::ProcessImage_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<color_shape_detector::srv::ProcessImage_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<color_shape_detector::srv::ProcessImage_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace color_shape_detector
{

namespace srv
{

inline void to_flow_style_yaml(
  const ProcessImage_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ProcessImage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ProcessImage_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace color_shape_detector

namespace rosidl_generator_traits
{

[[deprecated("use color_shape_detector::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const color_shape_detector::srv::ProcessImage_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  color_shape_detector::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use color_shape_detector::srv::to_yaml() instead")]]
inline std::string to_yaml(const color_shape_detector::srv::ProcessImage_Response & msg)
{
  return color_shape_detector::srv::to_yaml(msg);
}

template<>
inline const char * data_type<color_shape_detector::srv::ProcessImage_Response>()
{
  return "color_shape_detector::srv::ProcessImage_Response";
}

template<>
inline const char * name<color_shape_detector::srv::ProcessImage_Response>()
{
  return "color_shape_detector/srv/ProcessImage_Response";
}

template<>
struct has_fixed_size<color_shape_detector::srv::ProcessImage_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<color_shape_detector::srv::ProcessImage_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<color_shape_detector::srv::ProcessImage_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<color_shape_detector::srv::ProcessImage>()
{
  return "color_shape_detector::srv::ProcessImage";
}

template<>
inline const char * name<color_shape_detector::srv::ProcessImage>()
{
  return "color_shape_detector/srv/ProcessImage";
}

template<>
struct has_fixed_size<color_shape_detector::srv::ProcessImage>
  : std::integral_constant<
    bool,
    has_fixed_size<color_shape_detector::srv::ProcessImage_Request>::value &&
    has_fixed_size<color_shape_detector::srv::ProcessImage_Response>::value
  >
{
};

template<>
struct has_bounded_size<color_shape_detector::srv::ProcessImage>
  : std::integral_constant<
    bool,
    has_bounded_size<color_shape_detector::srv::ProcessImage_Request>::value &&
    has_bounded_size<color_shape_detector::srv::ProcessImage_Response>::value
  >
{
};

template<>
struct is_service<color_shape_detector::srv::ProcessImage>
  : std::true_type
{
};

template<>
struct is_service_request<color_shape_detector::srv::ProcessImage_Request>
  : std::true_type
{
};

template<>
struct is_service_response<color_shape_detector::srv::ProcessImage_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__TRAITS_HPP_
