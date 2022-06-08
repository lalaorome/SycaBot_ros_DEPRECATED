// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/Task.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__TASK__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__TASK__BUILDER_HPP_

#include "interfaces/srv/detail/task__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_Task_Request_id
{
public:
  Init_Task_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::Task_Request id(::interfaces::srv::Task_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::Task_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::Task_Request>()
{
  return interfaces::srv::builder::Init_Task_Request_id();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_Task_Response_task
{
public:
  Init_Task_Response_task()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::Task_Response task(::interfaces::srv::Task_Response::_task_type arg)
  {
    msg_.task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::Task_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::Task_Response>()
{
  return interfaces::srv::builder::Init_Task_Response_task();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__TASK__BUILDER_HPP_
