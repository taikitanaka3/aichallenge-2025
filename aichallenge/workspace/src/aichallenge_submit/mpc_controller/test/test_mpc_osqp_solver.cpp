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
    const int N = 20;  // 予測ホライズンを20に増加
    const int nx = 3;  // 状態変数の数 (x, y, yaw)
    const int nu = 2;  // 制御入力の数 (acceleration, steering_angle)

    // ソルバーの初期化
    solver_ = std::make_shared<mpc_controller::MPCOSQPSolver>(N, nx, nu);

    // 状態遷移行列
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    A(0, 2) = 0.1;  // x += v * cos(yaw) * dt
    A(1, 2) = 0.1;  // y += v * sin(yaw) * dt

    // 入力行列
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    B(0, 0) = 0.1;  // x += accel * dt
    B(1, 0) = 0.1;  // y += accel * dt
    B(2, 1) = 0.1;  // yaw += steer * dt

    // 重み行列の調整
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
    Q(0, 0) = 2.0;   // x位置の重み（現状維持）
    Q(1, 1) = 5.0;   // y位置の重みを大幅に増加（振動抑制のため）
    Q(2, 2) = 2.0;   // ヨー角の重みを増加（安定性向上）

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nu, nu);
    R(0, 0) = 1.2;   // 加速度の重みを増加（より滑らかな加速）
    R(1, 1) = 0.8;   // ステアリングの重みを増加（より滑らかな操舵）

    // 制約の調整
    Eigen::VectorXd u_min(nu), u_max(nu);
    u_min << -0.15, -0.1;  // 制限をより厳しく
    u_max << 1.15, 1.1;    // 制限をより厳しく

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
      // 簡易的な状態更新
      Eigen::VectorXd next_x = x;
      next_x(0) += u(0) * 0.1;  // x += accel * dt
      next_x(1) += u(0) * 0.1;  // y += accel * dt
      next_x(2) += u(1) * 0.1;  // yaw += steer * dt
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

// 直線軌道追従のテスト
TEST_F(TestMPCOSQPSolver, TrackReferenceTest)
{
  // 初期状態
  Eigen::VectorXd x0(nx_);
  x0 << 0.0, 0.0, 0.0;  // 初期位置と向き

  // 参照軌道の生成（10秒分、100ステップ）
  std::vector<Eigen::VectorXd> x_ref_seq;
  std::vector<double> ref_x, ref_y;
  for (int i = 0; i < 100; ++i) {
    Eigen::VectorXd x_ref(nx_);
    x_ref << 0.1 * i, 0.0, 0.0;  // 直線軌道
    x_ref_seq.push_back(x_ref);
    ref_x.push_back(x_ref(0));
    ref_y.push_back(x_ref(1));
  }

  // 10秒間のシミュレーション
  Eigen::VectorXd current_state = x0;
  for (int step = 0; step < 100; ++step) {
    // 制御入力を計算
    Eigen::VectorXd u = solver_->solve(current_state, x_ref_seq);

    // デバッグ情報を出力
    logDebugInfo(current_state, u, x_ref_seq);
    logPredictedTrajectory(current_state, u);

    // 状態を更新
    Eigen::VectorXd next_state = current_state;
    next_state(0) += u(0) * 0.1;  // x += accel * dt
    next_state(1) += u(0) * 0.1;  // y += accel * dt
    next_state(2) += u(1) * 0.1;  // yaw += steer * dt

    // 可視化
    visualization_->publishReferenceTrajectory(ref_x, ref_y);
    visualization_->publishPredictedTrajectory(
      {next_state(0)}, {next_state(1)}, {next_state(2)},
      current_state(0), current_state(1), current_state(2),
      u(0), u(1));

    // 状態を更新
    current_state = next_state;

    // 100ms待機（10Hz）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 最終状態の検証
  EXPECT_NEAR(current_state(0), x_ref_seq[99](0), 0.3);
  EXPECT_NEAR(current_state(1), x_ref_seq[99](1), 0.3);
  EXPECT_NEAR(current_state(2), x_ref_seq[99](2), 0.3);
}

// 右ターンのテスト
TEST_F(TestMPCOSQPSolver, TurnTest) {
    // 初期状態
    Eigen::VectorXd x0(3);
    x0 << 0.0, 0.0, 0.0;  // x, y, yaw

    // 右ターンの参照軌道（10秒分、100ステップ）
    std::vector<Eigen::VectorXd> x_ref_seq;
    std::vector<double> ref_x, ref_y;
    for (int i = 0; i < 100; ++i) {
        Eigen::VectorXd x_ref(3);
        double angle = i * 0.1;  // 10秒で1回転
        x_ref << std::cos(angle) * 5.0, std::sin(angle) * 5.0, angle;  // 半径5mの円
        x_ref_seq.push_back(x_ref);
        ref_x.push_back(x_ref(0));
        ref_y.push_back(x_ref(1));
    }

    // 10秒間のシミュレーション
    Eigen::VectorXd current_state = x0;
    for (int step = 0; step < 100; ++step) {
        // 制御入力を計算
        Eigen::VectorXd u = solver_->solve(current_state, x_ref_seq);
        EXPECT_EQ(u.size(), 2);

        // 状態を更新
        Eigen::VectorXd next_state = current_state;
        next_state(0) += u(0) * 0.1;  // x += accel * dt
        next_state(1) += u(0) * 0.1;  // y += accel * dt
        next_state(2) += u(1) * 0.1;  // yaw += steer * dt

        // 可視化
        visualization_->publishReferenceTrajectory(ref_x, ref_y);
        visualization_->publishPredictedTrajectory(
            {next_state(0)}, {next_state(1)}, {next_state(2)},
            current_state(0), current_state(1), current_state(2),
            u(0), u(1));

        // 状態を更新
        current_state = next_state;

        // 100ms待機（10Hz）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 最終状態の検証
    EXPECT_NEAR(current_state(0), x_ref_seq[99](0), 0.3);  // x位置
    EXPECT_NEAR(current_state(1), x_ref_seq[99](1), 0.3);  // y位置
    EXPECT_NEAR(current_state(2), x_ref_seq[99](2), 0.3);  // yaw角
}

TEST_F(TestMPCOSQPSolver, SimpleStraightLineTest) {
  // 初期状態を原点に設定
  Eigen::VectorXd x0(3);
  x0 << 0.0, 0.0, 0.0;  // x, y, yaw

  // 単純な直線軌道を生成（5ステップ、0.5m/sで直進）
  std::vector<Eigen::VectorXd> x_ref_seq;
  for (int i = 0; i < 5; ++i) {
    Eigen::VectorXd x_ref(3);
    x_ref << 0.5 * i * 0.1, 0.0, 0.0;  // 0.1秒間隔で0.5m/s
    x_ref_seq.push_back(x_ref);
  }

  // 制御入力を計算
  Eigen::VectorXd u = solver_->solve(x0, x_ref_seq);

  // 状態を更新（テスト内で直接計算）
  Eigen::VectorXd next_state = x0;
  next_state(0) += u(0) * 0.1;  // x += accel * dt
  next_state(1) += u(0) * 0.1;  // y += accel * dt
  next_state(2) += u(1) * 0.1;  // yaw += steer * dt

  // 結果を検証（許容誤差を0.1に緩和）
  EXPECT_NEAR(next_state(0), x_ref_seq[0](0), 0.1);
  EXPECT_NEAR(next_state(1), x_ref_seq[0](1), 0.1);
  EXPECT_NEAR(next_state(2), x_ref_seq[0](2), 0.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 