#include "custom_interfaces/action/waypoint.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TestWaypointAction : public ::testing::Test {
protected:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<custom_interfaces::action::Waypoint>::SharedPtr client_;
  double current_yaw_ = 0.0;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_waypoint_action");
    client_ = rclcpp_action::create_client<custom_interfaces::action::Waypoint>(
        node_, "tortoisebot_as");

    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(10)));

    // Subscribe to odometry and process data manually in a method.
    auto odom_subscription =
        node_->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&TestWaypointAction::processOdometry, this,
                      std::placeholders::_1));

    // Manually call processOdometry with test data
    nav_msgs::msg::Odometry test_msg;
    test_msg.pose.pose.orientation = tf2::toMsg(
        tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.785398)); // 45 degrees
    processOdometry(std::make_shared<nav_msgs::msg::Odometry>(test_msg));
  }

  void TearDown() override { rclcpp::shutdown(); }

  void processOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch;
    m.getRPY(roll, pitch, this->current_yaw_);
    RCLCPP_INFO(node_->get_logger(), "Current yaw updated: %f",
                this->current_yaw_);
  }
};

TEST_F(TestWaypointAction, TestGoalAchievement) {
  auto goal_msg = custom_interfaces::action::Waypoint::Goal();
  goal_msg.position.x = -5.0;
  goal_msg.position.y = -5.0;
  double expected_yaw =
      atan2(goal_msg.position.y - 0,
            goal_msg.position.x - 0); // Direct path, assuming starting at (0,0)

  bool goal_achieved = false;
  auto send_goal_options = rclcpp_action::Client<
      custom_interfaces::action::Waypoint>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, &goal_achieved, expected_yaw](
          const rclcpp_action::ClientGoalHandle<
              custom_interfaces::action::Waypoint>::WrappedResult &result) {
        ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
        ASSERT_TRUE(result.result->success);
        ASSERT_NEAR(this->current_yaw_, expected_yaw, 0.1);
        goal_achieved = true;
      };

  auto goal_handle_future =
      client_->async_send_goal(goal_msg, send_goal_options);
  rclcpp::spin_until_future_complete(node_, goal_handle_future,
                                     std::chrono::seconds(30));
  ASSERT_TRUE(goal_handle_future.get());

  // Ensure we continue to spin until the goal_achieved flag is set
  while (rclcpp::ok() && !goal_achieved) {
    rclcpp::spin_some(node_);
  }

  ASSERT_TRUE(goal_achieved);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
