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

#ifndef MPC_CONTROLLER_NODE_HPP_
#define MPC_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include "mpc_controller/vehicle_model.hpp"
#include "mpc_controller/visualization.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace mpc_controller
{

class MPCControllerNode : public rclcpp::Node
{
public:
  explicit MPCControllerNode();

private:
  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishControlCommand();

  // サブスクライバー
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  // パブリッシャー
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 車両モデル
  std::unique_ptr<LinearVehicleModel> vehicle_model_;

  // 可視化
  std::shared_ptr<Visualization> visualization_;

  // 最新のメッセージ
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr current_trajectory_;
  nav_msgs::msg::Odometry::SharedPtr current_odometry_;

  void publishVisualization(
    const std::vector<double>& predicted_x,
    const std::vector<double>& predicted_y,
    const std::vector<double>& predicted_yaw,
    const double current_x,
    const double current_y,
    const double current_yaw,
    const double acceleration,
    const double steering_angle);
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER_NODE_HPP_ 