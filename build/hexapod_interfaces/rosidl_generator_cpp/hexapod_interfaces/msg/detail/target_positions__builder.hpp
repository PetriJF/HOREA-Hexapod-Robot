// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__BUILDER_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hexapod_interfaces/msg/detail/target_positions__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hexapod_interfaces
{

namespace msg
{

namespace builder
{

class Init_TargetPositions_z_pos
{
public:
  explicit Init_TargetPositions_z_pos(::hexapod_interfaces::msg::TargetPositions & msg)
  : msg_(msg)
  {}
  ::hexapod_interfaces::msg::TargetPositions z_pos(::hexapod_interfaces::msg::TargetPositions::_z_pos_type arg)
  {
    msg_.z_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hexapod_interfaces::msg::TargetPositions msg_;
};

class Init_TargetPositions_y_pos
{
public:
  explicit Init_TargetPositions_y_pos(::hexapod_interfaces::msg::TargetPositions & msg)
  : msg_(msg)
  {}
  Init_TargetPositions_z_pos y_pos(::hexapod_interfaces::msg::TargetPositions::_y_pos_type arg)
  {
    msg_.y_pos = std::move(arg);
    return Init_TargetPositions_z_pos(msg_);
  }

private:
  ::hexapod_interfaces::msg::TargetPositions msg_;
};

class Init_TargetPositions_x_pos
{
public:
  Init_TargetPositions_x_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetPositions_y_pos x_pos(::hexapod_interfaces::msg::TargetPositions::_x_pos_type arg)
  {
    msg_.x_pos = std::move(arg);
    return Init_TargetPositions_y_pos(msg_);
  }

private:
  ::hexapod_interfaces::msg::TargetPositions msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hexapod_interfaces::msg::TargetPositions>()
{
  return hexapod_interfaces::msg::builder::Init_TargetPositions_x_pos();
}

}  // namespace hexapod_interfaces

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__BUILDER_HPP_
