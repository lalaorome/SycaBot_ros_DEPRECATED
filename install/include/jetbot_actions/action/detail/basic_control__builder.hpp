// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jetbot_actions:action/BasicControl.idl
// generated code does not contain a copyright notice

#ifndef JETBOT_ACTIONS__ACTION__DETAIL__BASIC_CONTROL__BUILDER_HPP_
#define JETBOT_ACTIONS__ACTION__DETAIL__BASIC_CONTROL__BUILDER_HPP_

#include "jetbot_actions/action/detail/basic_control__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_Goal_order
{
public:
  Init_BasicControl_Goal_order()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetbot_actions::action::BasicControl_Goal order(::jetbot_actions::action::BasicControl_Goal::_order_type arg)
  {
    msg_.order = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_Goal>()
{
  return jetbot_actions::action::builder::Init_BasicControl_Goal_order();
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_Result_achieved
{
public:
  Init_BasicControl_Result_achieved()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetbot_actions::action::BasicControl_Result achieved(::jetbot_actions::action::BasicControl_Result::_achieved_type arg)
  {
    msg_.achieved = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_Result>()
{
  return jetbot_actions::action::builder::Init_BasicControl_Result_achieved();
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_Feedback>()
{
  return ::jetbot_actions::action::BasicControl_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_SendGoal_Request_goal
{
public:
  explicit Init_BasicControl_SendGoal_Request_goal(::jetbot_actions::action::BasicControl_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::jetbot_actions::action::BasicControl_SendGoal_Request goal(::jetbot_actions::action::BasicControl_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_SendGoal_Request msg_;
};

class Init_BasicControl_SendGoal_Request_goal_id
{
public:
  Init_BasicControl_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicControl_SendGoal_Request_goal goal_id(::jetbot_actions::action::BasicControl_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_BasicControl_SendGoal_Request_goal(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_SendGoal_Request>()
{
  return jetbot_actions::action::builder::Init_BasicControl_SendGoal_Request_goal_id();
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_SendGoal_Response_stamp
{
public:
  explicit Init_BasicControl_SendGoal_Response_stamp(::jetbot_actions::action::BasicControl_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::jetbot_actions::action::BasicControl_SendGoal_Response stamp(::jetbot_actions::action::BasicControl_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_SendGoal_Response msg_;
};

class Init_BasicControl_SendGoal_Response_accepted
{
public:
  Init_BasicControl_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicControl_SendGoal_Response_stamp accepted(::jetbot_actions::action::BasicControl_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_BasicControl_SendGoal_Response_stamp(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_SendGoal_Response>()
{
  return jetbot_actions::action::builder::Init_BasicControl_SendGoal_Response_accepted();
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_GetResult_Request_goal_id
{
public:
  Init_BasicControl_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jetbot_actions::action::BasicControl_GetResult_Request goal_id(::jetbot_actions::action::BasicControl_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_GetResult_Request>()
{
  return jetbot_actions::action::builder::Init_BasicControl_GetResult_Request_goal_id();
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_GetResult_Response_result
{
public:
  explicit Init_BasicControl_GetResult_Response_result(::jetbot_actions::action::BasicControl_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::jetbot_actions::action::BasicControl_GetResult_Response result(::jetbot_actions::action::BasicControl_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_GetResult_Response msg_;
};

class Init_BasicControl_GetResult_Response_status
{
public:
  Init_BasicControl_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicControl_GetResult_Response_result status(::jetbot_actions::action::BasicControl_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_BasicControl_GetResult_Response_result(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_GetResult_Response>()
{
  return jetbot_actions::action::builder::Init_BasicControl_GetResult_Response_status();
}

}  // namespace jetbot_actions


namespace jetbot_actions
{

namespace action
{

namespace builder
{

class Init_BasicControl_FeedbackMessage_feedback
{
public:
  explicit Init_BasicControl_FeedbackMessage_feedback(::jetbot_actions::action::BasicControl_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::jetbot_actions::action::BasicControl_FeedbackMessage feedback(::jetbot_actions::action::BasicControl_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_FeedbackMessage msg_;
};

class Init_BasicControl_FeedbackMessage_goal_id
{
public:
  Init_BasicControl_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BasicControl_FeedbackMessage_feedback goal_id(::jetbot_actions::action::BasicControl_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_BasicControl_FeedbackMessage_feedback(msg_);
  }

private:
  ::jetbot_actions::action::BasicControl_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::jetbot_actions::action::BasicControl_FeedbackMessage>()
{
  return jetbot_actions::action::builder::Init_BasicControl_FeedbackMessage_goal_id();
}

}  // namespace jetbot_actions

#endif  // JETBOT_ACTIONS__ACTION__DETAIL__BASIC_CONTROL__BUILDER_HPP_
