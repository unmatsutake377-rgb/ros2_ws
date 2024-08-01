// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from color_shape_detector:srv/ProcessImage.idl
// generated code does not contain a copyright notice

#ifndef COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__BUILDER_HPP_
#define COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "color_shape_detector/srv/detail/process_image__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace color_shape_detector
{

namespace srv
{

namespace builder
{

class Init_ProcessImage_Request_request
{
public:
  Init_ProcessImage_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::color_shape_detector::srv::ProcessImage_Request request(::color_shape_detector::srv::ProcessImage_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::color_shape_detector::srv::ProcessImage_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::color_shape_detector::srv::ProcessImage_Request>()
{
  return color_shape_detector::srv::builder::Init_ProcessImage_Request_request();
}

}  // namespace color_shape_detector


namespace color_shape_detector
{

namespace srv
{

namespace builder
{

class Init_ProcessImage_Response_response
{
public:
  Init_ProcessImage_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::color_shape_detector::srv::ProcessImage_Response response(::color_shape_detector::srv::ProcessImage_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::color_shape_detector::srv::ProcessImage_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::color_shape_detector::srv::ProcessImage_Response>()
{
  return color_shape_detector::srv::builder::Init_ProcessImage_Response_response();
}

}  // namespace color_shape_detector

#endif  // COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__BUILDER_HPP_
