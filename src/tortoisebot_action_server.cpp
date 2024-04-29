// Copyright 2024 Miguel Gálvez
// Licensed under the Apache License, Version 2.0 (see LICENSE or
// http://www.apache.org/licenses/LICENSE-2.0)
// Additional license information may apply

#include "custom_interfaces/action/waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

class WaypointActionServer : public rclcpp::Node {
public:
  WaypointActionServer() : Node("waypoint_action_server"), _state("idle") {
    using namespace std::placeholders;

    _action_server =
        rclcpp_action::create_server<custom_interfaces::action::Waypoint>(
            this, "tortoisebot_as",
            std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
            std::bind(&WaypointActionServer::handle_cancel, this, _1),
            std::bind(&WaypointActionServer::handle_accepted, this, _1));

    _pub_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1));

    _yaw_precision = M_PI / 45;
    _dist_precision = 0.05;
  }

private:
  rclcpp_action::Server<custom_interfaces::action::Waypoint>::SharedPtr
      _action_server;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_cmd_vel;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom;
  geometry_msgs::msg::Point _position;
  double _yaw = 0.0;
  std::string _state;
  geometry_msgs::msg::Point _des_pos;
  double _yaw_precision;
  double _dist_precision;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    _position = msg->pose.pose.position;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    _yaw = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<
                  const custom_interfaces::action::Waypoint::Goal> /* goal */) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid; // To avoid unused variable warning
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<custom_interfaces::action::Waypoint>>
          goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle; // to avoid unused variable warning
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<custom_interfaces::action::Waypoint>>
          goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&WaypointActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void
  execute(const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<custom_interfaces::action::Waypoint>>
              goal_handle) {
    rclcpp::Rate loop_rate(25);
    const auto goal = goal_handle->get_goal();
    auto feedback =
        std::make_shared<custom_interfaces::action::Waypoint::Feedback>();
    auto result =
        std::make_shared<custom_interfaces::action::Waypoint::Result>();

    auto success = true;
    _des_pos = goal->position;
    auto desired_yaw =
        atan2(_des_pos.y - _position.y, _des_pos.x - _position.x);
    auto err_pos = sqrt(pow(_des_pos.y - _position.y, 2) +
                        pow(_des_pos.x - _position.x, 2));

    while (rclcpp::ok() && success) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        publishZeroVelocity();
        return;
      }

      desired_yaw = atan2(_des_pos.y - _position.y, _des_pos.x - _position.x);
      auto err_yaw = desired_yaw - _yaw;
      err_pos = sqrt(pow(_des_pos.y - _position.y, 2) +
                     pow(_des_pos.x - _position.x, 2));

      if (err_pos > _dist_precision) {
        if (fabs(err_yaw) > _yaw_precision) {
          RCLCPP_INFO(this->get_logger(), "Fixing yaw");
          _state = "fix yaw";
          auto twist_msg = geometry_msgs::msg::Twist();
          twist_msg.angular.z = std::clamp(15 * err_yaw, -0.65, 0.65);
          _pub_cmd_vel->publish(twist_msg);
        } else {
          RCLCPP_INFO(this->get_logger(), "Going to point");
          _state = "go to point";
          auto twist_msg = geometry_msgs::msg::Twist();
          twist_msg.linear.x = std::clamp(10 * err_pos, -0.85,
                                          0.85); // Adjust speed as necessary
          twist_msg.angular.z = 0;
          _pub_cmd_vel->publish(twist_msg);
        }
      } else {
        publishZeroVelocity(); // Stop the robot as it is within the precision
                               // limit
        break;                 // Exit the loop as the goal is reached
      }

      feedback->position = _position;
      feedback->state = _state;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    if (success) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      publishZeroVelocity(); // Make sure to stop the robot at the end
    }
  }

  void publishZeroVelocity() {
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    _pub_cmd_vel->publish(stop_msg);
    RCLCPP_INFO(this->get_logger(), "Stopping robot");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
