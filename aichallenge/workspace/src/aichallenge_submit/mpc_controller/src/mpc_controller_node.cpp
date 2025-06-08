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
#include "mpc_controller/visualization.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

  // RVizマーカーのパブリッシャー
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mpc_controller/visualization", 10);

  // 可視化オブジェクトの初期化
  visualization_ = std::make_shared<Visualization>(shared_from_this());
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

void MPCControllerNode::publishVisualization(
  const std::vector<double>& predicted_x,
  const std::vector<double>& predicted_y,
  const std::vector<double>& predicted_yaw,
  const double current_x,
  const double current_y,
  const double current_yaw,
  const double acceleration,
  const double steering_angle) {
  
  visualization_msgs::msg::MarkerArray marker_array;

  // 予測軌跡（青色のLine Strip）
  auto trajectory_marker = visualization_msgs::msg::Marker();
  trajectory_marker.header.frame_id = "map";
  trajectory_marker.header.stamp = this->now();
  trajectory_marker.ns = "predicted_trajectory";
  trajectory_marker.id = 0;
  trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
  trajectory_marker.scale.x = 0.1;  // 線の太さ
  trajectory_marker.color.r = 0.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;
  trajectory_marker.color.a = 1.0;

  for (size_t i = 0; i < predicted_x.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = predicted_x[i];
    p.y = predicted_y[i];
    p.z = 0.0;
    trajectory_marker.points.push_back(p);
  }
  marker_array.markers.push_back(trajectory_marker);

  // 車両の位置と向き（矢印）
  auto vehicle_marker = visualization_msgs::msg::Marker();
  vehicle_marker.header.frame_id = "map";
  vehicle_marker.header.stamp = this->now();
  vehicle_marker.ns = "vehicle";
  vehicle_marker.id = 1;
  vehicle_marker.type = visualization_msgs::msg::Marker::ARROW;
  vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
  vehicle_marker.scale.x = 1.0;  // 矢印の長さ
  vehicle_marker.scale.y = 0.2;  // 矢印の太さ
  vehicle_marker.scale.z = 0.2;  // 矢印の高さ
  vehicle_marker.color.r = 1.0;
  vehicle_marker.color.g = 0.0;
  vehicle_marker.color.b = 0.0;
  vehicle_marker.color.a = 1.0;

  // 矢印の開始点
  geometry_msgs::msg::Point start;
  start.x = current_x;
  start.y = current_y;
  start.z = 0.0;
  vehicle_marker.points.push_back(start);

  // 矢印の終点（進行方向）
  geometry_msgs::msg::Point end;
  end.x = current_x + std::cos(current_yaw);
  end.y = current_y + std::sin(current_yaw);
  end.z = 0.0;
  vehicle_marker.points.push_back(end);
  marker_array.markers.push_back(vehicle_marker);

  // 制御入力のテキスト表示
  auto text_marker = visualization_msgs::msg::Marker();
  text_marker.header.frame_id = "map";
  text_marker.header.stamp = this->now();
  text_marker.ns = "control_inputs";
  text_marker.id = 2;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.scale.z = 0.5;  // テキストのサイズ
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;

  text_marker.pose.position.x = current_x;
  text_marker.pose.position.y = current_y;
  text_marker.pose.position.z = 1.0;

  std::stringstream ss;
  ss << "Accel: " << std::fixed << std::setprecision(2) << acceleration << " m/s²\n"
     << "Steer: " << std::fixed << std::setprecision(2) << steering_angle * 180.0 / M_PI << "°";
  text_marker.text = ss.str();
  marker_array.markers.push_back(text_marker);

  marker_pub_->publish(marker_array);
}

}  // namespace mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpc_controller::MPCControllerNode>());
  rclcpp::shutdown();
  return 0;
} 