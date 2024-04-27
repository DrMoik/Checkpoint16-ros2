// Copyright 2024 Miguel Gálvez
// Licensed under the Apache License, Version 2.0 (see LICENSE or
// http://www.apache.org/licenses/LICENSE-2.0)
// Additional license information may apply

#include "custom_interfaces/action/waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class TestWaypointAction : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<custom_interfaces::action::Waypoint>::SharedPtr
      action_client;
  geometry_msgs::msg::Point current_position;
  double current_yaw;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_waypoint_action");
    action_client =
        rclcpp_action::create_client<custom_interfaces::action::Waypoint>(
            node, "tortoisebot_as");

    // Ensure the action server is available before proceeding
    ASSERT_TRUE(
        action_client->wait_for_action_server(std::chrono::seconds(10)));

    // Subscribe to the /odom topic to receive odometry information
    auto odom_subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          this->current_position = msg->pose.pose.position;
          auto &q = msg->pose.pose.orientation;
          tf2::Quaternion quat(q.x, q.y, q.z, q.w);
          tf2::Matrix3x3 m(quat);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          this->current_yaw = yaw;
        });
  }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestWaypointAction, TestPosition) {
  auto goal_msg = custom_interfaces::action::Waypoint::Goal();
  goal_msg.position.x = 0.5; // Ensure this is a realistic and achievable goal
  goal_msg.position.y = 0.5;

  auto send_goal_options = rclcpp_action::Client<
      custom_interfaces::action::Waypoint>::SendGoalOptions();
  send_goal_options.result_callback =
      [this,
       &goal_msg](const rclcpp_action::ClientGoalHandle<
                  custom_interfaces::action::Waypoint>::WrappedResult &result) {
        if (result.result->success) {
          EXPECT_NEAR(current_position.x, goal_msg.position.x, 0.1);
          EXPECT_NEAR(current_position.y, goal_msg.position.y, 0.1);
        } else {
          FAIL() << "Action failed";
        }
      };

  auto goal_handle_future =
      action_client->async_send_goal(goal_msg, send_goal_options);
  auto result = rclcpp::spin_until_future_complete(
      node, goal_handle_future,
      std::chrono::seconds(30)); // Ensure timeout is sufficient
  ASSERT_EQ(
      result,
      rclcpp::FutureReturnCode::SUCCESS); // Check if the result is successful

  std::this_thread::sleep_for(
      std::chrono::seconds(1)); // Give some extra time for the robot to settle
  rclcpp::spin_some(node);      // Process any remaining callbacks
}

TEST_F(TestWaypointAction, TestYaw) {
  auto goal_msg = custom_interfaces::action::Waypoint::Goal();
  goal_msg.position.x = 0.0; // Assuming this position change influences the yaw
  goal_msg.position.y = 0.0; // A direct north movement should change yaw

  double expected_yaw = std::atan2(goal_msg.position.y, goal_msg.position.x);

  auto send_goal_options = rclcpp_action::Client<
      custom_interfaces::action::Waypoint>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, expected_yaw](const rclcpp_action::ClientGoalHandle<
                           custom_interfaces::action::Waypoint>::WrappedResult &result) {
        if (result.result->success) {
          EXPECT_NEAR(current_yaw, expected_yaw, 0.3);
        } else {
          FAIL() << "Action failed";
        }
      };

  auto goal_handle_future =
      action_client->async_send_goal(goal_msg, send_goal_options);
  auto result = rclcpp::spin_until_future_complete(node, goal_handle_future,
                                                   std::chrono::seconds(30));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);

  rclcpp::spin_some(node); // Process any remaining callbacks
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
