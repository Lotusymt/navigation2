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

#include <memory>
#include <string>
#include <functional>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/managed_entity.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace nav2
{

/**
 * @brief A lifecycle-managed action server wrapper for Nav2
 *
 * This wrapper provides lifecycle management for action servers, similar to
 * nav2::Publisher and nav2::Subscription. It gates goal acceptance and execution
 * based on activation state. When deactivated, new goals are rejected and a
 * throttled warning is logged. For non-lifecycle nodes, the server is
 * automatically activated.
 */
template<typename ActionT>
class ActionServer : public rclcpp_lifecycle::SimpleManagedEntity
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(ActionServer)

  using RclcppServer = rclcpp_action::Server<ActionT>;

  // Callback types
  using GoalCallback = std::function<rclcpp_action::GoalResponse(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const typename ActionT::Goal>)>;
  using CancelCallback = std::function<rclcpp_action::CancelResponse(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>;
  using AcceptedCallback = std::function<void(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>;

  /**
   * @brief Constructor for ActionServer
   * @param node Node to create the action server on
   * @param action_name Name of the action
   * @param handle_goal Callback to handle goal requests
   * @param handle_cancel Callback to handle cancel requests
   * @param handle_accepted Callback to handle accepted goals
   * @param options Action server options (default options if not provided)
   * @param callback_group Callback group to use (optional)
   */
  template<typename NodeT>
  ActionServer(
    const std::shared_ptr<NodeT> & node,
    const std::string & action_name,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    AcceptedCallback handle_accepted,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
    rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
  : action_name_(action_name),
    logger_(node->get_logger()),
    clock_(node->get_clock())
  {
    init(node, handle_goal, handle_cancel, handle_accepted, options, callback_group);
  }

  void on_activate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_activate();
  }

  void on_deactivate() override
  {
    rclcpp_lifecycle::SimpleManagedEntity::on_deactivate();
  }

  const char * get_action_name() const noexcept
  {
    return action_name_.c_str();
  }

protected:
  template<typename NodeT>
  void init(
    const std::shared_ptr<NodeT> & node,
    GoalCallback handle_goal,
    CancelCallback handle_cancel,
    AcceptedCallback handle_accepted,
    const rcl_action_server_options_t & options,
    rclcpp::CallbackGroup::SharedPtr callback_group)
  {
    // Store user callbacks
    user_handle_goal_ = handle_goal;
    user_handle_cancel_ = handle_cancel;
    user_handle_accepted_ = handle_accepted;

    // Create wrapped callbacks that gate based on activation state
    auto wrapped_handle_goal =
      [this](const rclcpp_action::GoalUUID & uuid,
             std::shared_ptr<const typename ActionT::Goal> goal)
      {
        if (!this->is_activated()) {
          RCLCPP_WARN_THROTTLE(
            logger_,
            *clock_,
            1000,  // milliseconds -> 1Hz
            "ActionServer '%s' received goal before activation; rejecting until activated.",
            action_name_.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }

        // Call user callback if provided
        if (user_handle_goal_) {
          return user_handle_goal_(uuid, goal);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto wrapped_handle_cancel =
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
      {
        if (!this->is_activated()) {
          // Reject cancel if not activated (simplest behavior)
          return rclcpp_action::CancelResponse::REJECT;
        }

        // Call user callback if provided
        if (user_handle_cancel_) {
          return user_handle_cancel_(goal_handle);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto wrapped_handle_accepted =
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
      {
        if (!this->is_activated()) {
          // Don't execute if not activated
          return;
        }

        // Call user callback if provided
        if (user_handle_accepted_) {
          user_handle_accepted_(goal_handle);
        }
      };

    // Create the underlying action server
    server_ = rclcpp_action::create_server<ActionT>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name_,
      std::move(wrapped_handle_goal),
      std::move(wrapped_handle_cancel),
      std::move(wrapped_handle_accepted),
      options,
      callback_group);

    // Auto-activate for non-lifecycle nodes
    auto lc_node = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node);
    if (!lc_node) {
      on_activate();
    }
  }

  typename RclcppServer::SharedPtr server_;
  std::string action_name_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // User-provided callbacks
  GoalCallback user_handle_goal_;
  CancelCallback user_handle_cancel_;
  AcceptedCallback user_handle_accepted_;
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__ACTION_SERVER_HPP_
