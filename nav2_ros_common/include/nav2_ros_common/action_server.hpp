// Copyright (c) 2025 Open Navigation LLC
// Copyright (c) 2026 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_ROS_COMMON__ACTION_SERVER_HPP_
#define NAV2_ROS_COMMON__ACTION_SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"
#include "nav2_ros_common/simple_action_server.hpp"

namespace nav2
{

/**
 * @brief A lifecycle-managed action server wrapper for Nav2
 *
 * Wraps nav2::SimpleActionServer and extends SimpleManagedEntity so it can be
 * added to a LifecycleNode's managed entities. Activation and deactivation are
 * driven by lifecycle transitions; call sites no longer need to manually call
 * activate()/deactivate() in on_activate/on_deactivate.
 *
 * Exposes the full SimpleActionServer API via forwarding, so existing call
 * sites can use ActionServer as a drop-in replacement without code changes
 * (other than removing manual activate/deactivate calls).
 */
template<typename ActionT>
class ActionServer : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ActionServer)

  using SimpleServer = nav2::SimpleActionServer<ActionT>;

  template<typename NodeT>
  ActionServer(
    NodeT node,
    const std::string & action_name,
    typename SimpleServer::ExecuteCallback execute_cb,
    typename SimpleServer::CompletionCallback completion_cb = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const bool realtime = false)
  : simple_(std::make_shared<SimpleServer>(
        node, action_name, execute_cb, completion_cb, server_timeout, spin_thread, realtime))
  {}

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
    simple_->activate();
  }

  void on_deactivate() override
  {
    simple_->deactivate();
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
  }

  // --- Forward SimpleActionServer API (drop-in replacement for call sites) ---

  const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
  {
    return simple_->get_current_goal();
  }

  const rclcpp_action::GoalUUID get_current_goal_id() const
  {
    return simple_->get_current_goal_id();
  }

  const std::shared_ptr<const typename ActionT::Goal> get_pending_goal() const
  {
    return simple_->get_pending_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    return simple_->accept_pending_goal();
  }

  void terminate_pending_goal()
  {
    simple_->terminate_pending_goal();
  }

  void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    simple_->publish_feedback(feedback);
  }

  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    simple_->succeeded_current(result);
  }

  void terminate_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    simple_->terminate_current(result);
  }

  void terminate_all(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    simple_->terminate_all(result);
  }

  bool is_server_active()
  {
    return simple_->is_server_active();
  }

  bool is_cancel_requested() const
  {
    return simple_->is_cancel_requested();
  }

  bool is_preempt_requested() const
  {
    return simple_->is_preempt_requested();
  }

  bool is_running()
  {
    return simple_->is_running();
  }

  /**
   * @brief Direct access to the underlying SimpleActionServer
   *
   * Use when you need methods not forwarded here (e.g. setSoftRealTimePriority).
   * Prefer the forwarded methods for common operations.
   */
  SimpleServer & server() { return *simple_; }
  const SimpleServer & server() const { return *simple_; }

  /**
   * @brief Shared pointer to the underlying SimpleActionServer
   *
   * Useful when another component needs shared ownership.
   */
  std::shared_ptr<SimpleServer> get_simple_server() { return simple_; }
  std::shared_ptr<const SimpleServer> get_simple_server() const { return simple_; }

private:
  std::shared_ptr<SimpleServer> simple_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__ACTION_SERVER_HPP_
