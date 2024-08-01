// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from color_shape_detector:srv/ProcessImage.idl
// generated code does not contain a copyright notice

#ifndef COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__STRUCT_HPP_
#define COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__color_shape_detector__srv__ProcessImage_Request __attribute__((deprecated))
#else
# define DEPRECATED__color_shape_detector__srv__ProcessImage_Request __declspec(deprecated)
#endif

namespace color_shape_detector
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ProcessImage_Request_
{
  using Type = ProcessImage_Request_<ContainerAllocator>;

  explicit ProcessImage_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = 0l;
    }
  }

  explicit ProcessImage_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = 0l;
    }
  }

  // field types and members
  using _request_type =
    int32_t;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const int32_t & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__color_shape_detector__srv__ProcessImage_Request
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__color_shape_detector__srv__ProcessImage_Request
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ProcessImage_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const ProcessImage_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ProcessImage_Request_

// alias to use template instance with default allocator
using ProcessImage_Request =
  color_shape_detector::srv::ProcessImage_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace color_shape_detector


#ifndef _WIN32
# define DEPRECATED__color_shape_detector__srv__ProcessImage_Response __attribute__((deprecated))
#else
# define DEPRECATED__color_shape_detector__srv__ProcessImage_Response __declspec(deprecated)
#endif

namespace color_shape_detector
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ProcessImage_Response_
{
  using Type = ProcessImage_Response_<ContainerAllocator>;

  explicit ProcessImage_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = 0l;
    }
  }

  explicit ProcessImage_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = 0l;
    }
  }

  // field types and members
  using _response_type =
    int32_t;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const int32_t & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__color_shape_detector__srv__ProcessImage_Response
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__color_shape_detector__srv__ProcessImage_Response
    std::shared_ptr<color_shape_detector::srv::ProcessImage_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ProcessImage_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const ProcessImage_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ProcessImage_Response_

// alias to use template instance with default allocator
using ProcessImage_Response =
  color_shape_detector::srv::ProcessImage_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace color_shape_detector

namespace color_shape_detector
{

namespace srv
{

struct ProcessImage
{
  using Request = color_shape_detector::srv::ProcessImage_Request;
  using Response = color_shape_detector::srv::ProcessImage_Response;
};

}  // namespace srv

}  // namespace color_shape_detector

#endif  // COLOR_SHAPE_DETECTOR__SRV__DETAIL__PROCESS_IMAGE__STRUCT_HPP_
