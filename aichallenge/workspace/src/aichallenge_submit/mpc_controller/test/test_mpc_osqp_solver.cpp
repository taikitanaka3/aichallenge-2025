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

class TestMPCOSQPSolver : public ::testing::Test {
protected:
  void SetUp() override {
    // テスト用のパラメータ設定
    horizon_ = 10;
    state_dim_ = 4;  // 例: [x, y, theta, v]
    input_dim_ = 2;  // 例: [a, delta]

    // 状態空間モデル（単純な例）
    A_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    B_ = Eigen::MatrixXd::Zero(state_dim_, input_dim_);
    B_(0, 0) = 1.0;  // 加速度は位置に影響
    B_(1, 1) = 1.0;  // ステア角は向きに影響

    // 重み行列
    Q_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    R_ = 0.1 * Eigen::MatrixXd::Identity(input_dim_, input_dim_);

    // 制約
    u_min_ = Eigen::VectorXd::Constant(input_dim_, -1.0);
    u_max_ = Eigen::VectorXd::Constant(input_dim_, 1.0);

    // ソルバの初期化
    solver_ = std::make_unique<mpc_controller::MPCOSQPSolver>(horizon_, state_dim_, input_dim_);
    solver_->setModel(A_, B_);
    solver_->setWeight(Q_, R_);
    solver_->setConstraint(u_min_, u_max_);
  }

  int horizon_;
  int state_dim_;
  int input_dim_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;
  std::unique_ptr<mpc_controller::MPCOSQPSolver> solver_;
};

TEST_F(TestMPCOSQPSolver, TrackReferenceTest) {
  // 現在状態
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(state_dim_);
  x0(0) = 0.0;  // x
  x0(1) = 0.0;  // y
  x0(2) = 0.0;  // theta
  x0(3) = 0.0;  // v

  // 目標状態列（直進）
  std::vector<Eigen::VectorXd> x_ref_seq;
  for (int i = 0; i < horizon_; ++i) {
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(state_dim_);
    x_ref(0) = i * 0.1;  // x: 0.1ずつ進む
    x_ref(1) = 0.0;      // y: 0
    x_ref(2) = 0.0;      // theta: 0
    x_ref(3) = 0.1;      // v: 0.1
    x_ref_seq.push_back(x_ref);
  }

  // MPCを解く
  Eigen::VectorXd u = solver_->solve(x0, x_ref_seq);

  // 結果の検証
  EXPECT_EQ(u.size(), input_dim_);
  EXPECT_GE(u(0), u_min_(0));  // 加速度制約
  EXPECT_LE(u(0), u_max_(0));
  EXPECT_GE(u(1), u_min_(1));  // ステア角制約
  EXPECT_LE(u(1), u_max_(1));

  // 直進なので、ステア角は0に近いはず
  EXPECT_NEAR(u(1), 0.0, 0.1);
}

TEST_F(TestMPCOSQPSolver, TurnTest) {
  // 現在状態
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(state_dim_);
  x0(0) = 0.0;  // x
  x0(1) = 0.0;  // y
  x0(2) = 0.0;  // theta
  x0(3) = 0.1;  // v

  // 目標状態列（右折）
  std::vector<Eigen::VectorXd> x_ref_seq;
  for (int i = 0; i < horizon_; ++i) {
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(state_dim_);
    x_ref(0) = 0.1 * std::cos(i * 0.1);  // x: 円弧
    x_ref(1) = 0.1 * std::sin(i * 0.1);  // y: 円弧
    x_ref(2) = i * 0.1;                  // theta: 回転
    x_ref(3) = 0.1;                      // v: 一定
    x_ref_seq.push_back(x_ref);
  }

  // MPCを解く
  Eigen::VectorXd u = solver_->solve(x0, x_ref_seq);

  // 結果の検証
  EXPECT_EQ(u.size(), input_dim_);
  EXPECT_GE(u(0), u_min_(0));  // 加速度制約
  EXPECT_LE(u(0), u_max_(0));
  EXPECT_GE(u(1), u_min_(1));  // ステア角制約
  EXPECT_LE(u(1), u_max_(1));

  // 右折なので、ステア角は正のはず
  EXPECT_GT(u(1), 0.0);
} 