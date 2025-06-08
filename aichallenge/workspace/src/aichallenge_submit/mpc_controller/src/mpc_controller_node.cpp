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

#include "mpc_controller/mpc_controller_node.hpp"

namespace mpc_controller
{

MPCControllerNode::MPCControllerNode()
: Node("mpc_controller_node")
{
  // パラメータの設定
  const double wheelbase = this->declare_parameter("wheelbase", 2.7);
  const double steer_ratio = this->declare_parameter("steer_ratio", 14.8);
  const double steer_angle_velocity = this->declare_parameter("steer_angle_velocity", 0.5);
  const double steer_angle_limit = this->declare_parameter("steer_angle_limit", 0.5);
  const double acceleration_limit = this->declare_parameter("acceleration_limit", 3.0);

  // 車両モデルの初期化
  vehicle_model_ = std::make_unique<LinearVehicleModel>(wheelbase, steer_angle_limit, steer_angle_velocity);

  // サブスクライバーの設定
  trajectory_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", 1,
    std::bind(&MPCControllerNode::onTrajectory, this, std::placeholders::_1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 1,
    std::bind(&MPCControllerNode::onOdometry, this, std::placeholders::_1));

  // パブリッシャーの設定
  control_cmd_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1);
}

void MPCControllerNode::onTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  current_trajectory_ = msg;
  publishControlCommand();
}

void MPCControllerNode::onOdometry(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odometry_ = msg;
  publishControlCommand();
}

void MPCControllerNode::publishControlCommand()
{
  if (!current_trajectory_ || !current_odometry_) {
    return;
  }

  // 制御コマンドの作成
  auto control_cmd = std::make_unique<autoware_auto_control_msgs::msg::AckermannControlCommand>();
  control_cmd->stamp = this->now();
  control_cmd->lateral.steering_tire_angle = 0.0;  // 仮の値
  control_cmd->longitudinal.acceleration = 0.0;    // 仮の値

  // 制御コマンドのパブリッシュ
  control_cmd_pub_->publish(std::move(control_cmd));
}

}  // namespace mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpc_controller::MPCControllerNode>());
  rclcpp::shutdown();
  return 0;
} 