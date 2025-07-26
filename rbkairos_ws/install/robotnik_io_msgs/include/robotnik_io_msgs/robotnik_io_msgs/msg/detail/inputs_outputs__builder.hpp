// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_io_msgs:msg/InputsOutputs.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__BUILDER_HPP_
#define ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_io_msgs/msg/detail/inputs_outputs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_io_msgs
{

namespace msg
{

namespace builder
{

class Init_InputsOutputs_analog_outputs
{
public:
  explicit Init_InputsOutputs_analog_outputs(::robotnik_io_msgs::msg::InputsOutputs & msg)
  : msg_(msg)
  {}
  ::robotnik_io_msgs::msg::InputsOutputs analog_outputs(::robotnik_io_msgs::msg::InputsOutputs::_analog_outputs_type arg)
  {
    msg_.analog_outputs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_io_msgs::msg::InputsOutputs msg_;
};

class Init_InputsOutputs_analog_inputs
{
public:
  explicit Init_InputsOutputs_analog_inputs(::robotnik_io_msgs::msg::InputsOutputs & msg)
  : msg_(msg)
  {}
  Init_InputsOutputs_analog_outputs analog_inputs(::robotnik_io_msgs::msg::InputsOutputs::_analog_inputs_type arg)
  {
    msg_.analog_inputs = std::move(arg);
    return Init_InputsOutputs_analog_outputs(msg_);
  }

private:
  ::robotnik_io_msgs::msg::InputsOutputs msg_;
};

class Init_InputsOutputs_digital_outputs
{
public:
  explicit Init_InputsOutputs_digital_outputs(::robotnik_io_msgs::msg::InputsOutputs & msg)
  : msg_(msg)
  {}
  Init_InputsOutputs_analog_inputs digital_outputs(::robotnik_io_msgs::msg::InputsOutputs::_digital_outputs_type arg)
  {
    msg_.digital_outputs = std::move(arg);
    return Init_InputsOutputs_analog_inputs(msg_);
  }

private:
  ::robotnik_io_msgs::msg::InputsOutputs msg_;
};

class Init_InputsOutputs_digital_inputs
{
public:
  Init_InputsOutputs_digital_inputs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InputsOutputs_digital_outputs digital_inputs(::robotnik_io_msgs::msg::InputsOutputs::_digital_inputs_type arg)
  {
    msg_.digital_inputs = std::move(arg);
    return Init_InputsOutputs_digital_outputs(msg_);
  }

private:
  ::robotnik_io_msgs::msg::InputsOutputs msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_io_msgs::msg::InputsOutputs>()
{
  return robotnik_io_msgs::msg::builder::Init_InputsOutputs_digital_inputs();
}

}  // namespace robotnik_io_msgs

#endif  // ROBOTNIK_IO_MSGS__MSG__DETAIL__INPUTS_OUTPUTS__BUILDER_HPP_
