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
#include <vector>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "mpc_controller/mpc_osqp_solver.hpp"
#include "mpc_controller/vehicle_model.hpp"
#include "mpc_controller/visualization.hpp"
#include <random>
#include <iostream>
#include <fstream>

class TestMPCOSQPSolver : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_mpc_osqp_solver");
    visualization_ = std::make_shared<mpc_controller::Visualization>(node_);

    // デバッグ用のログファイルを開く
    debug_log_.open("mpc_debug.log");
    if (!debug_log_.is_open()) {
      std::cerr << "Failed to open debug log file" << std::endl;
    }

    // MPCのパラメータ設定
    const int N = 20;  // 予測ホライズン
    const int nx = 4;  // 状態変数の数 (x, y, yaw, v)
    const int nu = 2;  // 制御入力の数 (acceleration, steering_angle)

    // ソルバーの初期化
    solver_ = std::make_shared<mpc_controller::MPCOSQPSolver>(N, nx, nu);

    // 車両モデルのパラメータ
    double wheel_base = 2.5;  // 仮の値
    double steer_lim = 0.3;   // テストの入力制約と合わせる
    double steer_rate_lim = 0.5; // 仮の値
    mpc_controller::LinearVehicleModel vehicle_model(wheel_base, steer_lim, steer_rate_lim);

    // 状態遷移行列
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    double dt = 0.1;  // 時間ステップ
    double v = 1.0;   // 初期速度
    double yaw = 0.0; // 初期ヨー角
    A(0, 2) = -v * std::sin(yaw) * dt;  // x += v * cos(yaw) * dt
    A(1, 2) = v * std::cos(yaw) * dt;   // y += v * sin(yaw) * dt
    A(0, 3) = std::cos(yaw) * dt;       // x += v * cos(yaw) * dt
    A(1, 3) = std::sin(yaw) * dt;       // y += v * sin(yaw) * dt

    // 入力行列
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    B(2, 1) = dt;     // yaw += steer * dt
    B(3, 0) = dt;     // v += accel * dt

    // 重み行列の調整
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
    Q(0, 0) = 20.0;  // x位置の重みを増加
    Q(1, 1) = 20.0;  // y位置の重みを増加
    Q(2, 2) = 10.0;  // ヨー角の重みを増加
    Q(3, 3) = 5.0;   // 速度の重み

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nu, nu);
    R(0, 0) = 0.05;  // 加速度の重みを減少
    R(1, 1) = 0.05;  // ステアリングの重みを減少

    // 制約の調整
    Eigen::VectorXd u_min(nu), u_max(nu);
    u_min << -0.5, -0.3;  // 制限を緩和
    u_max << 0.5, 0.3;    // 制限を緩和

    // パラメータの設定
    solver_->setModel(A, B);
    solver_->setWeight(Q, R);
    solver_->setConstraint(u_min, u_max);

    // 乱数生成器の初期化
    std::random_device rd;
    gen_ = std::mt19937(rd());
    dist_ = std::normal_distribution<>(0.0, 0.05);  // 標準偏差を0.05に減少

    // クラスメンバとして保存
    N_ = N;
    nx_ = nx;
    nu_ = nu;
  }

  void TearDown() override
  {
    if (debug_log_.is_open()) {
      debug_log_.close();
    }
    rclcpp::shutdown();
  }

  // デバッグ情報を出力する関数
  void logDebugInfo(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const std::vector<Eigen::VectorXd>& x_ref_seq)
  {
    if (!debug_log_.is_open()) return;

    debug_log_ << "Current State: " << x.transpose() << std::endl;
    debug_log_ << "Control Input: " << u.transpose() << std::endl;
    debug_log_ << "Reference States:" << std::endl;
    for (size_t i = 0; i < x_ref_seq.size(); ++i) {
      debug_log_ << "  Step " << i << ": " << x_ref_seq[i].transpose() << std::endl;
    }
    debug_log_ << "------------------------" << std::endl;
  }

  // 予測軌跡を出力する関数
  void logPredictedTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& u)
  {
    if (!debug_log_.is_open()) return;

    debug_log_ << "Predicted Trajectory:" << std::endl;
    Eigen::VectorXd x = x0;
    debug_log_ << "  Initial: " << x.transpose() << std::endl;

    for (int i = 0; i < N_; ++i) {
      // 正確な状態更新
      Eigen::VectorXd next_x = x;
      double dt = 0.1;
      double v = 1.0;  // 現在の速度
      next_x(0) += v * std::cos(x(2)) * dt + u(0) * std::cos(x(2)) * dt * dt / 2.0;
      next_x(1) += v * std::sin(x(2)) * dt + u(0) * std::sin(x(2)) * dt * dt / 2.0;
      next_x(2) += u(1) * dt;
      debug_log_ << "  Step " << i + 1 << ": " << next_x.transpose() << std::endl;
      x = next_x;
    }
    debug_log_ << "------------------------" << std::endl;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<mpc_controller::Visualization> visualization_;
  std::shared_ptr<mpc_controller::MPCOSQPSolver> solver_;
  std::mt19937 gen_;
  std::normal_distribution<> dist_;
  std::ofstream debug_log_;  // デバッグログファイル
  int N_;  // 予測ホライズン
  int nx_;  // 状態次元
  int nu_;  // 入力次元
};

// 右ターンのテスト
TEST_F(TestMPCOSQPSolver, TurnTest) {
    // 初期状態を目標軌道の始点に合わせる
    Eigen::VectorXd x0(4);
    x0 << 5.0, 0.0, 0.0, 1.0;  // x, y, yaw, v（円の始点）

    // 右ターンの参照軌道（10秒分、100ステップ）
    std::vector<Eigen::VectorXd> x_ref_seq;
    std::vector<double> ref_x, ref_y;
    for (int i = 0; i < 100; ++i) {
        Eigen::VectorXd x_ref(4);
        double angle = i * 0.1;  // 10秒で1ラジアンずつ進む
        x_ref << std::cos(angle) * 5.0, std::sin(angle) * 5.0, angle, 1.0;  // 半径5mの円
        x_ref_seq.push_back(x_ref);
        ref_x.push_back(x_ref(0));
        ref_y.push_back(x_ref(1));
    }

    // 10秒間のシミュレーション
    Eigen::VectorXd current_state = x0;
    for (int step = 0; step < 100; ++step) {
        // 現在状態・入力からA,B行列を線形化
        Eigen::MatrixXd A, B, C, W;
        double steer = 0.0; // 目標軌道が円なので、理想的には一定値。ここでは0で近似。
        mpc_controller::LinearVehicleModel vehicle_model(2.5, 0.3, 0.5);
        vehicle_model.calculateStateSpaceMatrix(current_state, steer, A, B, C, W);
        solver_->setModel(A, B);

        // 制御入力を計算
        Eigen::VectorXd u = solver_->solve(current_state, x_ref_seq);
        EXPECT_EQ(u.size(), 2);

        // 状態を更新
        Eigen::VectorXd next_state = current_state;
        double dt = 0.1;
        double v = current_state(3);  // 現在の速度
        double yaw = current_state(2);
        next_state(0) += v * std::cos(yaw) * dt + u(0) * std::cos(yaw) * dt * dt / 2.0;
        next_state(1) += v * std::sin(yaw) * dt + u(0) * std::sin(yaw) * dt * dt / 2.0;
        next_state(2) += u(1) * dt;
        next_state(3) += u(0) * dt;  // 速度の更新

        // 予測軌跡を1ステップ分だけ可視化（next_state）
        std::vector<double> pred_x = {next_state(0)};
        std::vector<double> pred_y = {next_state(1)};
        std::vector<double> pred_yaw = {next_state(2)};
        visualization_->publishReferenceTrajectory(ref_x, ref_y);
        visualization_->publishPredictedTrajectory(
            pred_x, pred_y, pred_yaw,
            current_state(0), current_state(1), current_state(2),
            u(0), u(1));

        // 状態を更新
        current_state = next_state;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 最終状態の検証
    EXPECT_NEAR(current_state(0), x_ref_seq[99](0), 0.3);
    EXPECT_NEAR(current_state(1), x_ref_seq[99](1), 0.3);
    EXPECT_NEAR(current_state(2), x_ref_seq[99](2), 0.3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 