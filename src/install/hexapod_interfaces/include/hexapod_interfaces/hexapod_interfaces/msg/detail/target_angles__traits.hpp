// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hexapod_interfaces:msg/TargetAngles.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__TRAITS_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hexapod_interfaces/msg/detail/target_angles__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hexapod_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetAngles & msg,
  std::ostream & out)
{
  out << "{";
  // member: shoulder_angle
  {
    if (msg.shoulder_angle.size() == 0) {
      out << "shoulder_angle: []";
    } else {
      out << "shoulder_angle: [";
      size_t pending_items = msg.shoulder_angle.size();
      for (auto item : msg.shoulder_angle) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: hip_angle
  {
    if (msg.hip_angle.size() == 0) {
      out << "hip_angle: []";
    } else {
      out << "hip_angle: [";
      size_t pending_items = msg.hip_angle.size();
      for (auto item : msg.hip_angle) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: knee_angle
  {
    if (msg.knee_angle.size() == 0) {
      out << "knee_angle: []";
    } else {
      out << "knee_angle: [";
      size_t pending_items = msg.knee_angle.size();
      for (auto item : msg.knee_angle) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetAngles & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: shoulder_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.shoulder_angle.size() == 0) {
      out << "shoulder_angle: []\n";
    } else {
      out << "shoulder_angle:\n";
      for (auto item : msg.shoulder_angle) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: hip_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.hip_angle.size() == 0) {
      out << "hip_angle: []\n";
    } else {
      out << "hip_angle:\n";
      for (auto item : msg.hip_angle) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: knee_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.knee_angle.size() == 0) {
      out << "knee_angle: []\n";
    } else {
      out << "knee_angle:\n";
      for (auto item : msg.knee_angle) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetAngles & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hexapod_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hexapod_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hexapod_interfaces::msg::TargetAngles & msg,
  std::ostream & out, size_t indentation = 0)
{
  hexapod_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hexapod_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hexapod_interfaces::msg::TargetAngles & msg)
{
  return hexapod_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hexapod_interfaces::msg::TargetAngles>()
{
  return "hexapod_interfaces::msg::TargetAngles";
}

template<>
inline const char * name<hexapod_interfaces::msg::TargetAngles>()
{
  return "hexapod_interfaces/msg/TargetAngles";
}

template<>
struct has_fixed_size<hexapod_interfaces::msg::TargetAngles>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hexapod_interfaces::msg::TargetAngles>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hexapod_interfaces::msg::TargetAngles>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_ANGLES__TRAITS_HPP_
