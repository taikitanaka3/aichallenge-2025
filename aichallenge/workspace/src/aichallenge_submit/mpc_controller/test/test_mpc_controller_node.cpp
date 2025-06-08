// Copyright 2024 Taiki Tanaka
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

#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

using namespace std::chrono_literals;

class TestMPCControllerNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_mpc_controller_node");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

// トピックの存在確認テスト
TEST_F(TestMPCControllerNode, TopicExistenceTest)
{
  // パブリッシャーとサブスクライバーの作成
  auto trajectory_pub = node_->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", 1);
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 1);
  auto control_cmd_sub = node_->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1,
    [](const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr) {});

  // パブリッシャーとサブスクライバーが正しく作成されたことを確認
  EXPECT_TRUE(trajectory_pub != nullptr);
  EXPECT_TRUE(odom_pub != nullptr);
  EXPECT_TRUE(control_cmd_sub != nullptr);

  // トピックの存在確認
  auto topics = node_->get_topic_names_and_types();
  bool has_trajectory_topic = false;
  bool has_odom_topic = false;
  bool has_control_cmd_topic = false;

  for (const auto & topic : topics) {
    if (topic.first == "/planning/trajectory") has_trajectory_topic = true;
    if (topic.first == "/localization/kinematic_state") has_odom_topic = true;
    if (topic.first == "/control/command/control_cmd") has_control_cmd_topic = true;
  }

  EXPECT_TRUE(has_trajectory_topic);
  EXPECT_TRUE(has_odom_topic);
  EXPECT_TRUE(has_control_cmd_topic);
}

// メッセージのパブリッシュ/サブスクライブテスト
TEST_F(TestMPCControllerNode, MessagePublishSubscribeTest)
{
  // 制御コマンドのサブスクライバー
  bool received_control_cmd = false;
  auto control_cmd_sub = node_->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1,
    [&received_control_cmd](const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg) {
      received_control_cmd = true;
      // 制御コマンドの値が適切な範囲内にあることを確認
      EXPECT_GE(msg->lateral.steering_tire_angle, -0.5);  // ステアリング角制限
      EXPECT_LE(msg->lateral.steering_tire_angle, 0.5);
      EXPECT_GE(msg->longitudinal.acceleration, -3.0);    // 加速度制限
      EXPECT_LE(msg->longitudinal.acceleration, 3.0);
    });

  // テスト用の軌道をパブリッシュ
  auto trajectory_pub = node_->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", 1);
  auto trajectory_msg = std::make_unique<autoware_auto_planning_msgs::msg::Trajectory>();
  // 軌道の設定（テスト用の簡単な軌道）
  trajectory_msg->points.resize(10);
  for (size_t i = 0; i < 10; ++i) {
    trajectory_msg->points[i].pose.position.x = static_cast<double>(i);
    trajectory_msg->points[i].pose.position.y = 0.0;
    trajectory_msg->points[i].longitudinal_velocity_mps = 10.0;
  }
  trajectory_pub->publish(std::move(trajectory_msg));

  // テスト用の車両状態（Odometry）をパブリッシュ
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 1);
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->pose.pose.position.x = 0.0;
  odom_msg->pose.pose.position.y = 0.0;
  odom_msg->pose.pose.orientation.w = 1.0;  // ヨー角0度
  odom_msg->twist.twist.linear.x = 10.0;    // 速度10 m/s
  odom_pub->publish(std::move(odom_msg));

  // メッセージが処理されるのを待つ
  rclcpp::Rate rate(10);
  for (int i = 0; i < 50 && !received_control_cmd; ++i) {  // 待機時間を5秒に延長
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  EXPECT_TRUE(received_control_cmd);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 