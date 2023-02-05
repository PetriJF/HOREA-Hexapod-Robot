// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__STRUCT_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hexapod_interfaces__msg__TargetAngles __attribute__((deprecated))
#else
# define DEPRECATED__hexapod_interfaces__msg__TargetAngles __declspec(deprecated)
#endif

namespace hexapod_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetAngles_
{
  using Type = TargetAngles_<ContainerAllocator>;

  explicit TargetAngles_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->shoulder_angle.begin(), this->shoulder_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->hip_angle.begin(), this->hip_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->knee_angle.begin(), this->knee_angle.end(), 0.0f);
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<float, 7>::iterator, float>(this->shoulder_angle.begin(), this->shoulder_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->hip_angle.begin(), this->hip_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->knee_angle.begin(), this->knee_angle.end(), 0.0f);
    }
  }

  explicit TargetAngles_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : shoulder_angle(_alloc),
    hip_angle(_alloc),
    knee_angle(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->shoulder_angle.begin(), this->shoulder_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->hip_angle.begin(), this->hip_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->knee_angle.begin(), this->knee_angle.end(), 0.0f);
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<float, 7>::iterator, float>(this->shoulder_angle.begin(), this->shoulder_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->hip_angle.begin(), this->hip_angle.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->knee_angle.begin(), this->knee_angle.end(), 0.0f);
    }
  }

  // field types and members
  using _shoulder_angle_type =
    std::array<float, 7>;
  _shoulder_angle_type shoulder_angle;
  using _hip_angle_type =
    std::array<float, 7>;
  _hip_angle_type hip_angle;
  using _knee_angle_type =
    std::array<float, 7>;
  _knee_angle_type knee_angle;

  // setters for named parameter idiom
  Type & set__shoulder_angle(
    const std::array<float, 7> & _arg)
  {
    this->shoulder_angle = _arg;
    return *this;
  }
  Type & set__hip_angle(
    const std::array<float, 7> & _arg)
  {
    this->hip_angle = _arg;
    return *this;
  }
  Type & set__knee_angle(
    const std::array<float, 7> & _arg)
  {
    this->knee_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hexapod_interfaces::msg::TargetAngles_<ContainerAllocator> *;
  using ConstRawPtr =
    const hexapod_interfaces::msg::TargetAngles_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hexapod_interfaces::msg::TargetAngles_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hexapod_interfaces::msg::TargetAngles_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hexapod_interfaces__msg__TargetAngles
    std::shared_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hexapod_interfaces__msg__TargetAngles
    std::shared_ptr<hexapod_interfaces::msg::TargetAngles_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetAngles_ & other) const
  {
    if (this->shoulder_angle != other.shoulder_angle) {
      return false;
    }
    if (this->hip_angle != other.hip_angle) {
      return false;
    }
    if (this->knee_angle != other.knee_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetAngles_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetAngles_

// alias to use template instance with default allocator
using TargetAngles =
  hexapod_interfaces::msg::TargetAngles_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hexapod_interfaces

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__STRUCT_HPP_
