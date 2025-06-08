// Copyright 2025 Taiki Tanaka
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
#include "mpc_controller/vehicle_model.hpp"

using mpc_controller::LinearVehicleModel;

class TestLinearVehicleModel : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用のパラメータ
    const double wheel_base = 2.7;      // ホイールベース [m]
    const double steer_lim = 0.5;       // ステアリング角制限 [rad]
    const double steer_rate_lim = 0.2;  // ステアリング角速度制限 [rad/s]

    model_ = std::make_unique<LinearVehicleModel>(wheel_base, steer_lim, steer_rate_lim);
  }

  std::unique_ptr<LinearVehicleModel> model_;
};

// 状態更新のテスト
TEST_F(TestLinearVehicleModel, StateUpdateTest)
{
  // 初期状態
  Eigen::VectorXd x(4);
  x << 0.0, 0.0, 0.0, 10.0;  // [x, y, yaw, v]

  // 入力
  const double dt = 0.1;     // 時間ステップ [s]
  const double steer = 0.1;  // ステアリング角 [rad]
  const double accel = 0.0;  // 加速度 [m/s^2]

  // 状態更新
  Eigen::VectorXd next_x(4);
  model_->updateState(x, dt, steer, accel, next_x);

  // 期待値の計算
  const double expected_x = 10.0 * dt;  // x = v * cos(yaw) * dt
  const double expected_y = 0.0;        // y = v * sin(yaw) * dt
  const double expected_yaw = 10.0 * std::tan(steer) / 2.7 * dt;  // yaw = v * tan(steer) / wheel_base * dt
  const double expected_v = 10.0;       // v = v + accel * dt

  // テスト
  EXPECT_NEAR(next_x(0), expected_x, 1e-6);
  EXPECT_NEAR(next_x(1), expected_y, 1e-6);
  EXPECT_NEAR(next_x(2), expected_yaw, 1e-6);
  EXPECT_NEAR(next_x(3), expected_v, 1e-6);
}

// 状態空間モデルの行列計算のテスト
TEST_F(TestLinearVehicleModel, StateSpaceMatrixTest)
{
  // 状態
  Eigen::VectorXd x(4);
  x << 0.0, 0.0, 0.0, 10.0;  // [x, y, yaw, v]

  // ステアリング角
  const double steer = 0.1;  // [rad]

  // 行列の計算
  Eigen::MatrixXd A, B, C, W;  // 状態空間モデルの行列
  // A: 状態行列 (4x4) - システムの状態遷移を表す
  // B: 入力行列 (4x2) - 制御入力（ステアリング角、加速度）の影響を表す
  // C: 出力行列 (4x1) - システムの出力を表す（このモデルでは使用しない）
  // W: 定数項 (4x1) - 非線形項の線形化による定数項
  model_->calculateStateSpaceMatrix(x, steer, A, B, C, W);

  // テスト
  // A行列のテスト
  EXPECT_EQ(A.rows(), 4);
  EXPECT_EQ(A.cols(), 4);
  EXPECT_NEAR(A(0, 2), 0.0, 1e-6);  // -v * sin(yaw)
  EXPECT_NEAR(A(0, 3), 1.0, 1e-6);  // cos(yaw)
  EXPECT_NEAR(A(1, 2), 10.0, 1e-6);  // v * cos(yaw)
  EXPECT_NEAR(A(1, 3), 0.0, 1e-6);  // sin(yaw)
  EXPECT_NEAR(A(2, 3), std::tan(steer) / 2.7, 1e-6);  // tan(steer) / wheel_base

  // B行列のテスト
  EXPECT_EQ(B.rows(), 4);
  EXPECT_EQ(B.cols(), 2);
  EXPECT_NEAR(B(2, 0), 10.0 / (2.7 * std::cos(steer) * std::cos(steer)), 1e-6);
  EXPECT_NEAR(B(3, 1), 1.0, 1e-6);

  // C行列のテスト
  EXPECT_EQ(C.rows(), 4);
  EXPECT_EQ(C.cols(), 1);
  // C行列は零行列であることを確認
  EXPECT_NEAR(C(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(C(1, 0), 0.0, 1e-6);
  EXPECT_NEAR(C(2, 0), 0.0, 1e-6);
  EXPECT_NEAR(C(3, 0), 0.0, 1e-6);
  // 別の方法でも零行列であることを確認
  EXPECT_TRUE(C.isZero());

  // W行列のテスト
  EXPECT_EQ(W.rows(), 4);
  EXPECT_EQ(W.cols(), 1);
  EXPECT_NEAR(W(0), 0.0, 1e-6);
  EXPECT_NEAR(W(1), 0.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
} 