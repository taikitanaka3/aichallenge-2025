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
#include "mpc_controller/mpc_osqp_solver.hpp"
#include "mpc_controller/visualization.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

class TestMPCOSQPSolver : public ::testing::Test {
protected:
  void SetUp() override {
    // ノードの初期化
    node_ = std::make_shared<rclcpp::Node>("test_mpc_solver");
    visualization_ = std::make_shared<mpc_controller::Visualization>(node_);

    // MPCソルバーのパラメータ設定
    const int N = 10;  // 予測ホライズン
    const int nx = 3;  // 状態変数の数 (x, y, yaw)
    const int nu = 2;  // 制御入力の数 (acceleration, steering)

    // 状態遷移行列
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    A(0, 2) = 0.1;  // x += v * cos(yaw) * dt
    A(1, 2) = 0.1;  // y += v * sin(yaw) * dt

    // 入力行列
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    B(0, 0) = 0.1;  // x += accel * dt
    B(1, 0) = 0.1;  // y += accel * dt
    B(2, 1) = 0.1;  // yaw += steer * dt

    // 重み行列
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nu, nu);
    R *= 0.1;  // 制御入力の重みを小さく

    // 制約
    Eigen::VectorXd x_min(nx), x_max(nx);
    x_min << -100, -100, -M_PI;
    x_max << 100, 100, M_PI;

    Eigen::VectorXd u_min(nu), u_max(nu);
    u_min << -3.0, -0.5;  // 加速度制限: -3.0 m/s², ステアリング制限: -0.5 rad
    u_max << 3.0, 0.5;    // 加速度制限: 3.0 m/s², ステアリング制限: 0.5 rad

    // ソルバーの初期化
    solver_ = std::make_shared<mpc_controller::MPCOSQPSolver>(
      N, nx, nu, A, B, Q, R, x_min, x_max, u_min, u_max);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<mpc_controller::Visualization> visualization_;
  std::shared_ptr<mpc_controller::MPCOSQPSolver> solver_;
};

// 直線軌道追従のテスト
TEST_F(TestMPCOSQPSolver, TrackReferenceTest) {
  // 現在の状態
  std::vector<double> current_state = {0.0, 0.0, 0.0};  // x, y, yaw

  // 参照軌道（直線）
  std::vector<std::vector<double>> reference_trajectory;
  for (int i = 0; i < 100; ++i) {
    reference_trajectory.push_back({i * 0.1, 0.0, 0.0});  // x, y, yaw
  }

  // 10秒間のシミュレーション
  const double dt = 0.1;  // 制御周期
  const int sim_steps = 100;  // 10秒 / 0.1秒 = 100ステップ

  for (int step = 0; step < sim_steps; ++step) {
    // MPCソルバーで制御入力を計算
    std::vector<double> control_inputs = solver_->solve(current_state, reference_trajectory);

    // 予測軌跡を計算（簡易的な実装）
    std::vector<double> predicted_x, predicted_y, predicted_yaw;
    double x = current_state[0];
    double y = current_state[1];
    double yaw = current_state[2];
    double v = 0.0;  // 初期速度

    for (int i = 0; i < 10; ++i) {  // 予測ホライズン分
      v += control_inputs[0] * dt;  // 加速度による速度更新
      x += v * std::cos(yaw) * dt;
      y += v * std::sin(yaw) * dt;
      yaw += control_inputs[1] * dt;

      predicted_x.push_back(x);
      predicted_y.push_back(y);
      predicted_yaw.push_back(yaw);
    }

    // 可視化
    visualization_->publishTrajectory(
      predicted_x, predicted_y, predicted_yaw,
      current_state[0], current_state[1], current_state[2],
      control_inputs[0], control_inputs[1]);

    // 状態を更新
    v += control_inputs[0] * dt;
    current_state[0] += v * std::cos(current_state[2]) * dt;
    current_state[1] += v * std::sin(current_state[2]) * dt;
    current_state[2] += control_inputs[1] * dt;

    // 制御周期分待機
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// 右ターンのテスト
TEST_F(TestMPCOSQPSolver, TurnTest) {
  // 現在の状態
  std::vector<double> current_state = {0.0, 0.0, 0.0};  // x, y, yaw

  // 参照軌道（右ターン）
  std::vector<std::vector<double>> reference_trajectory;
  const double radius = 10.0;
  for (int i = 0; i < 100; ++i) {
    double angle = i * 0.1;
    reference_trajectory.push_back({
      radius * (1.0 - std::cos(angle)),  // x
      radius * std::sin(angle),          // y
      angle                              // yaw
    });
  }

  // 10秒間のシミュレーション
  const double dt = 0.1;  // 制御周期
  const int sim_steps = 100;  // 10秒 / 0.1秒 = 100ステップ

  for (int step = 0; step < sim_steps; ++step) {
    // MPCソルバーで制御入力を計算
    std::vector<double> control_inputs = solver_->solve(current_state, reference_trajectory);

    // 予測軌跡を計算（簡易的な実装）
    std::vector<double> predicted_x, predicted_y, predicted_yaw;
    double x = current_state[0];
    double y = current_state[1];
    double yaw = current_state[2];
    double v = 0.0;  // 初期速度

    for (int i = 0; i < 10; ++i) {  // 予測ホライズン分
      v += control_inputs[0] * dt;  // 加速度による速度更新
      x += v * std::cos(yaw) * dt;
      y += v * std::sin(yaw) * dt;
      yaw += control_inputs[1] * dt;

      predicted_x.push_back(x);
      predicted_y.push_back(y);
      predicted_yaw.push_back(yaw);
    }

    // 可視化
    visualization_->publishTrajectory(
      predicted_x, predicted_y, predicted_yaw,
      current_state[0], current_state[1], current_state[2],
      control_inputs[0], control_inputs[1]);

    // 状態を更新
    v += control_inputs[0] * dt;
    current_state[0] += v * std::cos(current_state[2]) * dt;
    current_state[1] += v * std::sin(current_state[2]) * dt;
    current_state[2] += control_inputs[1] * dt;

    // 制御周期分待機
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 