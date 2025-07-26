// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotnik_navigation_msgs:action/Move.idl
// generated code does not contain a copyright notice

#ifndef ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__MOVE__BUILDER_HPP_
#define ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotnik_navigation_msgs/action/detail/move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_Goal_maximum_velocity
{
public:
  explicit Init_Move_Goal_maximum_velocity(::robotnik_navigation_msgs::action::Move_Goal & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Move_Goal maximum_velocity(::robotnik_navigation_msgs::action::Move_Goal::_maximum_velocity_type arg)
  {
    msg_.maximum_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_Goal msg_;
};

class Init_Move_Goal_goal
{
public:
  Init_Move_Goal_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_Goal_maximum_velocity goal(::robotnik_navigation_msgs::action::Move_Goal::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_Move_Goal_maximum_velocity(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_Goal>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_Goal_goal();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_Result_response
{
public:
  Init_Move_Result_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_navigation_msgs::action::Move_Result response(::robotnik_navigation_msgs::action::Move_Result::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_Result>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_Result_response();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_Feedback_current_velocity
{
public:
  explicit Init_Move_Feedback_current_velocity(::robotnik_navigation_msgs::action::Move_Feedback & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Move_Feedback current_velocity(::robotnik_navigation_msgs::action::Move_Feedback::_current_velocity_type arg)
  {
    msg_.current_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_Feedback msg_;
};

class Init_Move_Feedback_remaining
{
public:
  Init_Move_Feedback_remaining()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_Feedback_current_velocity remaining(::robotnik_navigation_msgs::action::Move_Feedback::_remaining_type arg)
  {
    msg_.remaining = std::move(arg);
    return Init_Move_Feedback_current_velocity(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_Feedback>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_Feedback_remaining();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_SendGoal_Request_goal
{
public:
  explicit Init_Move_SendGoal_Request_goal(::robotnik_navigation_msgs::action::Move_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Move_SendGoal_Request goal(::robotnik_navigation_msgs::action::Move_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_SendGoal_Request msg_;
};

class Init_Move_SendGoal_Request_goal_id
{
public:
  Init_Move_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_SendGoal_Request_goal goal_id(::robotnik_navigation_msgs::action::Move_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Move_SendGoal_Request_goal(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_SendGoal_Request>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_SendGoal_Request_goal_id();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_SendGoal_Response_stamp
{
public:
  explicit Init_Move_SendGoal_Response_stamp(::robotnik_navigation_msgs::action::Move_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Move_SendGoal_Response stamp(::robotnik_navigation_msgs::action::Move_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_SendGoal_Response msg_;
};

class Init_Move_SendGoal_Response_accepted
{
public:
  Init_Move_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_SendGoal_Response_stamp accepted(::robotnik_navigation_msgs::action::Move_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Move_SendGoal_Response_stamp(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_SendGoal_Response>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_SendGoal_Response_accepted();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_GetResult_Request_goal_id
{
public:
  Init_Move_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotnik_navigation_msgs::action::Move_GetResult_Request goal_id(::robotnik_navigation_msgs::action::Move_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_GetResult_Request>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_GetResult_Request_goal_id();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_GetResult_Response_result
{
public:
  explicit Init_Move_GetResult_Response_result(::robotnik_navigation_msgs::action::Move_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Move_GetResult_Response result(::robotnik_navigation_msgs::action::Move_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_GetResult_Response msg_;
};

class Init_Move_GetResult_Response_status
{
public:
  Init_Move_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_GetResult_Response_result status(::robotnik_navigation_msgs::action::Move_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Move_GetResult_Response_result(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_GetResult_Response>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_GetResult_Response_status();
}

}  // namespace robotnik_navigation_msgs


namespace robotnik_navigation_msgs
{

namespace action
{

namespace builder
{

class Init_Move_FeedbackMessage_feedback
{
public:
  explicit Init_Move_FeedbackMessage_feedback(::robotnik_navigation_msgs::action::Move_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robotnik_navigation_msgs::action::Move_FeedbackMessage feedback(::robotnik_navigation_msgs::action::Move_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_FeedbackMessage msg_;
};

class Init_Move_FeedbackMessage_goal_id
{
public:
  Init_Move_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Move_FeedbackMessage_feedback goal_id(::robotnik_navigation_msgs::action::Move_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Move_FeedbackMessage_feedback(msg_);
  }

private:
  ::robotnik_navigation_msgs::action::Move_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotnik_navigation_msgs::action::Move_FeedbackMessage>()
{
  return robotnik_navigation_msgs::action::builder::Init_Move_FeedbackMessage_goal_id();
}

}  // namespace robotnik_navigation_msgs

#endif  // ROBOTNIK_NAVIGATION_MSGS__ACTION__DETAIL__MOVE__BUILDER_HPP_
