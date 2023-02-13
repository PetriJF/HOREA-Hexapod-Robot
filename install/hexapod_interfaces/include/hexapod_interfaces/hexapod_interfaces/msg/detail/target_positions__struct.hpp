// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__STRUCT_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hexapod_interfaces__msg__TargetPositions __attribute__((deprecated))
#else
# define DEPRECATED__hexapod_interfaces__msg__TargetPositions __declspec(deprecated)
#endif

namespace hexapod_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetPositions_
{
  using Type = TargetPositions_<ContainerAllocator>;

  explicit TargetPositions_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->x_pos.begin(), this->x_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->y_pos.begin(), this->y_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->z_pos.begin(), this->z_pos.end(), 0.0f);
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<float, 7>::iterator, float>(this->x_pos.begin(), this->x_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->y_pos.begin(), this->y_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->z_pos.begin(), this->z_pos.end(), 0.0f);
    }
  }

  explicit TargetPositions_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : x_pos(_alloc),
    y_pos(_alloc),
    z_pos(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->x_pos.begin(), this->x_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->y_pos.begin(), this->y_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->z_pos.begin(), this->z_pos.end(), 0.0f);
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      std::fill<typename std::array<float, 7>::iterator, float>(this->x_pos.begin(), this->x_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->y_pos.begin(), this->y_pos.end(), 0.0f);
      std::fill<typename std::array<float, 7>::iterator, float>(this->z_pos.begin(), this->z_pos.end(), 0.0f);
    }
  }

  // field types and members
  using _x_pos_type =
    std::array<float, 7>;
  _x_pos_type x_pos;
  using _y_pos_type =
    std::array<float, 7>;
  _y_pos_type y_pos;
  using _z_pos_type =
    std::array<float, 7>;
  _z_pos_type z_pos;

  // setters for named parameter idiom
  Type & set__x_pos(
    const std::array<float, 7> & _arg)
  {
    this->x_pos = _arg;
    return *this;
  }
  Type & set__y_pos(
    const std::array<float, 7> & _arg)
  {
    this->y_pos = _arg;
    return *this;
  }
  Type & set__z_pos(
    const std::array<float, 7> & _arg)
  {
    this->z_pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hexapod_interfaces::msg::TargetPositions_<ContainerAllocator> *;
  using ConstRawPtr =
    const hexapod_interfaces::msg::TargetPositions_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hexapod_interfaces::msg::TargetPositions_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hexapod_interfaces::msg::TargetPositions_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hexapod_interfaces__msg__TargetPositions
    std::shared_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hexapod_interfaces__msg__TargetPositions
    std::shared_ptr<hexapod_interfaces::msg::TargetPositions_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetPositions_ & other) const
  {
    if (this->x_pos != other.x_pos) {
      return false;
    }
    if (this->y_pos != other.y_pos) {
      return false;
    }
    if (this->z_pos != other.z_pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetPositions_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetPositions_

// alias to use template instance with default allocator
using TargetPositions =
  hexapod_interfaces::msg::TargetPositions_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hexapod_interfaces

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__STRUCT_HPP_
