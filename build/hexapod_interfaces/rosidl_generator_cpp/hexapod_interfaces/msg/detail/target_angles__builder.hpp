// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__BUILDER_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hexapod_interfaces/msg/detail/target_angles__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hexapod_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetAngles_knee_angle
{
public:
  explicit Init_TargetAngles_knee_angle(::hexapod_interfaces::msg::TargetAngles & msg)
  : msg_(msg)
  {}
  ::hexapod_interfaces::msg::TargetAngles knee_angle(::hexapod_interfaces::msg::TargetAngles::_knee_angle_type arg)
  {
    msg_.knee_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hexapod_interfaces::msg::TargetAngles msg_;
};

class Init_TargetAngles_hip_angle
{
public:
  explicit Init_TargetAngles_hip_angle(::hexapod_interfaces::msg::TargetAngles & msg)
  : msg_(msg)
  {}
  Init_TargetAngles_knee_angle hip_angle(::hexapod_interfaces::msg::TargetAngles::_hip_angle_type arg)
  {
    msg_.hip_angle = std::move(arg);
    return Init_TargetAngles_knee_angle(msg_);
  }

private:
  ::hexapod_interfaces::msg::TargetAngles msg_;
};

class Init_TargetAngles_shoulder_angle
{
public:
  Init_TargetAngles_shoulder_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetAngles_hip_angle shoulder_angle(::hexapod_interfaces::msg::TargetAngles::_shoulder_angle_type arg)
  {
    msg_.shoulder_angle = std::move(arg);
    return Init_TargetAngles_hip_angle(msg_);
  }

private:
  ::hexapod_interfaces::msg::TargetAngles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hexapod_interfaces::msg::TargetAngles>()
{
  return hexapod_interfaces::msg::builder::Init_TargetAngles_shoulder_angle();
}

}  // namespace hexapod_interfaces

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__BUILDER_HPP_
