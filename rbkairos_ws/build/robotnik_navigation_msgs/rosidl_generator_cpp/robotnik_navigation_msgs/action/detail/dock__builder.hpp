// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_navigation_msgs:action/Dock.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__DOCK__BUILDER_HPP_
#define ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__DOCK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_navigation_msgs/action/detail/dock__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_Goal_maximum_velocity
{
public:
  explicit Init_Dock_Goal_maximum_velocity(::robotnik_navigation_msgs::action::Dock_Goal & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Dock_Goal maximum_velocity(::robotnik_navigation_msgs::action::Dock_Goal::_maximum_velocity_type arg)
  {
    msg_.maximum_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Goal msg_;
};

class Init_Dock_Goal_dock_offset
{
public:
  explicit Init_Dock_Goal_dock_offset(::robotnik_navigation_msgs::action::Dock_Goal & msg)
  : msg_(msg)
  {}
  Init_Dock_Goal_maximum_velocity dock_offset(::robotnik_navigation_msgs::action::Dock_Goal::_dock_offset_type arg)
  {
    msg_.dock_offset = std::move(arg);
    return Init_Dock_Goal_maximum_velocity(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Goal msg_;
};

class Init_Dock_Goal_robot_dock_frame
{
public:
  explicit Init_Dock_Goal_robot_dock_frame(::robotnik_navigation_msgs::action::Dock_Goal & msg)
  : msg_(msg)
  {}
  Init_Dock_Goal_dock_offset robot_dock_frame(::robotnik_navigation_msgs::action::Dock_Goal::_robot_dock_frame_type arg)
  {
    msg_.robot_dock_frame = std::move(arg);
    return Init_Dock_Goal_dock_offset(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Goal msg_;
};

class Init_Dock_Goal_dock_frame
{
public:
  Init_Dock_Goal_dock_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_Goal_robot_dock_frame dock_frame(::robotnik_navigation_msgs::action::Dock_Goal::_dock_frame_type arg)
  {
    msg_.dock_frame = std::move(arg);
    return Init_Dock_Goal_robot_dock_frame(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_Goal>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_Goal_dock_frame();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_Result_response
{
public:
  Init_Dock_Result_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_navigation_msgs::action::Dock_Result response(::robotnik_navigation_msgs::action::Dock_Result::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_Result>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_Result_response();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_Feedback_current_velocity
{
public:
  explicit Init_Dock_Feedback_current_velocity(::robotnik_navigation_msgs::action::Dock_Feedback & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Dock_Feedback current_velocity(::robotnik_navigation_msgs::action::Dock_Feedback::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Feedback msg_;
};

class Init_Dock_Feedback_remaining
{
public:
  Init_Dock_Feedback_remaining()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_Feedback_current_velocity remaining(::robotnik_navigation_msgs::action::Dock_Feedback::_remaining_type arg)
  {
    msg_.remaining = std::move(arg);
    return Init_Dock_Feedback_current_velocity(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_Feedback>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_Feedback_remaining();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_SendGoal_Request_goal
{
public:
  explicit Init_Dock_SendGoal_Request_goal(::robotnik_navigation_msgs::action::Dock_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Dock_SendGoal_Request goal(::robotnik_navigation_msgs::action::Dock_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_SendGoal_Request msg_;
};

class Init_Dock_SendGoal_Request_goal_id
{
public:
  Init_Dock_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_SendGoal_Request_goal goal_id(::robotnik_navigation_msgs::action::Dock_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Dock_SendGoal_Request_goal(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_SendGoal_Request>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_SendGoal_Request_goal_id();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_SendGoal_Response_stamp
{
public:
  explicit Init_Dock_SendGoal_Response_stamp(::robotnik_navigation_msgs::action::Dock_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Dock_SendGoal_Response stamp(::robotnik_navigation_msgs::action::Dock_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_SendGoal_Response msg_;
};

class Init_Dock_SendGoal_Response_accepted
{
public:
  Init_Dock_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_SendGoal_Response_stamp accepted(::robotnik_navigation_msgs::action::Dock_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Dock_SendGoal_Response_stamp(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_SendGoal_Response>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_SendGoal_Response_accepted();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_GetResult_Request_goal_id
{
public:
  Init_Dock_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_navigation_msgs::action::Dock_GetResult_Request goal_id(::robotnik_navigation_msgs::action::Dock_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_GetResult_Request>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_GetResult_Request_goal_id();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_GetResult_Response_result
{
public:
  explicit Init_Dock_GetResult_Response_result(::robotnik_navigation_msgs::action::Dock_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Dock_GetResult_Response result(::robotnik_navigation_msgs::action::Dock_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_GetResult_Response msg_;
};

class Init_Dock_GetResult_Response_status
{
public:
  Init_Dock_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_GetResult_Response_result status(::robotnik_navigation_msgs::action::Dock_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Dock_GetResult_Response_result(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_GetResult_Response>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_GetResult_Response_status();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Dock_FeedbackMessage_feedback
{
public:
  explicit Init_Dock_FeedbackMessage_feedback(::robotnik_navigation_msgs::action::Dock_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Dock_FeedbackMessage feedback(::robotnik_navigation_msgs::action::Dock_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_FeedbackMessage msg_;
};

class Init_Dock_FeedbackMessage_goal_id
{
public:
  Init_Dock_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dock_FeedbackMessage_feedback goal_id(::robotnik_navigation_msgs::action::Dock_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Dock_FeedbackMessage_feedback(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Dock_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Dock_FeedbackMessage>()
{
  return robotnik_navigation_msgs::action::builder::Init_Dock_FeedbackMessage_goal_id();
}

}  // namespace robotnik_navigation_msgs

#endif  // ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__DOCK__BUILDER_HPP_
