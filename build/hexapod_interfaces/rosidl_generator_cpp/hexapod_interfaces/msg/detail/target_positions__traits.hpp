// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hexapod_interfaces:msg/TargetPositions.idl
// generated code does not contain a copyright notice

#ifndef HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__TRAITS_HPP_
#define HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hexapod_interfaces/msg/detail/target_positions__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hexapod_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetPositions & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_pos
  {
    if (msg.x_pos.size() == 0) {
      out << "x_pos: []";
    } else {
      out << "x_pos: [";
      size_t pending_items = msg.x_pos.size();
      for (auto item : msg.x_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y_pos
  {
    if (msg.y_pos.size() == 0) {
      out << "y_pos: []";
    } else {
      out << "y_pos: [";
      size_t pending_items = msg.y_pos.size();
      for (auto item : msg.y_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: z_pos
  {
    if (msg.z_pos.size() == 0) {
      out << "z_pos: []";
    } else {
      out << "z_pos: [";
      size_t pending_items = msg.z_pos.size();
      for (auto item : msg.z_pos) {
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
  const TargetPositions & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x_pos.size() == 0) {
      out << "x_pos: []\n";
    } else {
      out << "x_pos:\n";
      for (auto item : msg.x_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y_pos.size() == 0) {
      out << "y_pos: []\n";
    } else {
      out << "y_pos:\n";
      for (auto item : msg.y_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: z_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.z_pos.size() == 0) {
      out << "z_pos: []\n";
    } else {
      out << "z_pos:\n";
      for (auto item : msg.z_pos) {
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

inline std::string to_yaml(const TargetPositions & msg, bool use_flow_style = false)
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
  const hexapod_interfaces::msg::TargetPositions & msg,
  std::ostream & out, size_t indentation = 0)
{
  hexapod_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hexapod_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hexapod_interfaces::msg::TargetPositions & msg)
{
  return hexapod_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hexapod_interfaces::msg::TargetPositions>()
{
  return "hexapod_interfaces::msg::TargetPositions";
}

template<>
inline const char * name<hexapod_interfaces::msg::TargetPositions>()
{
  return "hexapod_interfaces/msg/TargetPositions";
}

template<>
struct has_fixed_size<hexapod_interfaces::msg::TargetPositions>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hexapod_interfaces::msg::TargetPositions>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hexapod_interfaces::msg::TargetPositions>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HEXAPOD_INTERFACES__MSG__DETAIL__TARGET_POSITIONS__TRAITS_HPP_
