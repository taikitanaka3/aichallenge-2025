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

class TestMPCOSQPSolver : public rclcpp::Node
{
public:
  TestMPCOSQPSolver()
  : Node("test_mpc_osqp_solver")
  {
    // パラメータの設定
    const double dt = 0.1;
    const int prediction_horizon = 10;
    const double max_steer = 0.5;
    const double min_steer = -0.5;
    const double max_accel = 2.0;
    const double min_accel = -2.0;
    const double max_steer_rate = 0.1;
    const double max_accel_rate = 0.1;
    const double q_x = 1.0;
    const double q_y = 1.0;
    const double q_yaw = 1.0;
    const double q_steer = 0.1;
    const double q_accel = 0.1;
    const double q_steer_rate = 0.1;
    const double q_accel_rate = 0.1;

    // 車両モデルの初期化
    vehicle_model_ = std::make_shared<VehicleModel>(dt);

    // MPCソルバーの初期化
    mpc_solver_ = std::make_shared<MPCOSQPSolver>(
      prediction_horizon,
      max_steer, min_steer,
      max_accel, min_accel,
      max_steer_rate, max_accel_rate,
      q_x, q_y, q_yaw,
      q_steer, q_accel,
      q_steer_rate, q_accel_rate,
      vehicle_model_);

    // 可視化の初期化
    visualization_ = std::make_shared<Visualization>(shared_from_this());

    // テスト実行
    runTest();
  }

private:
  void runTest()
  {
    // 現在の状態
    std::vector<double> current_state = {0.0, 0.0, 0.0, 0.0, 0.0};  // x, y, yaw, v, steer

    // 参照軌跡（直線）
    std::vector<double> reference_x;
    std::vector<double> reference_y;
    for (double x = 0.0; x <= 10.0; x += 0.1) {
      reference_x.push_back(x);
      reference_y.push_back(0.0);
    }

    // 参照軌跡を可視化
    visualization_->publishReferenceTrajectory(reference_x, reference_y);

    // MPCを解く
    std::vector<double> control_inputs;
    std::vector<double> predicted_x;
    std::vector<double> predicted_y;
    std::vector<double> predicted_yaw;

    auto start = std::chrono::high_resolution_clock::now();
    bool success = mpc_solver_->solve(
      current_state,
      reference_x, reference_y,
      control_inputs,
      predicted_x, predicted_y, predicted_yaw);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "MPC solved successfully in %ld ms", duration.count());
      RCLCPP_INFO(this->get_logger(), "Control inputs: accel = %f, steer = %f",
        control_inputs[0], control_inputs[1]);

      // 可視化
      visualization_->publishPredictedTrajectory(
        predicted_x, predicted_y, predicted_yaw,
        current_state[0], current_state[1], current_state[2],
        control_inputs[0], control_inputs[1]);
    } else {
      RCLCPP_ERROR(this->get_logger(), "MPC failed to solve");
    }
  }

  std::shared_ptr<VehicleModel> vehicle_model_;
  std::shared_ptr<MPCOSQPSolver> mpc_solver_;
  std::shared_ptr<Visualization> visualization_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestMPCOSQPSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 