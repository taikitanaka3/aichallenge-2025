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

#include "mpc_controller/vehicle_model.hpp"

namespace mpc_controller
{

LinearVehicleModel::LinearVehicleModel(
  const double wheel_base, const double steer_lim, const double steer_rate_lim)
: wheel_base_(wheel_base), steer_lim_(steer_lim), steer_rate_lim_(steer_rate_lim)
{
}

void LinearVehicleModel::updateState(
  const Eigen::VectorXd & x, const double dt, const double steer, const double accel,
  Eigen::VectorXd & next_x)
{
  // 状態ベクトル x = [x, y, yaw, v]
  const double x_curr = x(0);
  const double y_curr = x(1);
  const double yaw_curr = x(2);
  const double v_curr = x(3);

  // 状態更新
  next_x(0) = x_curr + v_curr * std::cos(yaw_curr) * dt;
  next_x(1) = y_curr + v_curr * std::sin(yaw_curr) * dt;
  next_x(2) = yaw_curr + v_curr * std::tan(steer) / wheel_base_ * dt;
  next_x(3) = v_curr + accel * dt;
}

void LinearVehicleModel::calculateStateSpaceMatrix(
  const Eigen::VectorXd & x, const double steer, Eigen::MatrixXd & A, Eigen::MatrixXd & B,
  Eigen::MatrixXd & C, Eigen::MatrixXd & W)
{
  const double v = x(3);
  const double yaw = x(2);

  // 状態空間モデルの行列を計算
  // x(k+1) = A * x(k) + B * u(k) + C * d(k) + W
  // ここで、x = [x, y, yaw, v], u = [steer, accel], d = [0] (外乱なし)

  // A行列 (状態遷移行列)
  A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(0, 2) = -v * std::sin(yaw);
  A(0, 3) = std::cos(yaw);
  A(1, 2) = v * std::cos(yaw);
  A(1, 3) = std::sin(yaw);
  A(2, 3) = std::tan(steer) / wheel_base_;

  // B行列 (入力行列)
  B = Eigen::MatrixXd::Zero(dim_x_, dim_u_);
  B(2, 0) = v / (wheel_base_ * std::cos(steer) * std::cos(steer));
  B(3, 1) = 1.0;

  // C行列 (外乱行列)
  C = Eigen::MatrixXd::Zero(dim_x_, 1);

  // W行列 (定数項)
  W = Eigen::VectorXd::Zero(dim_x_);
  W(0) = v * std::sin(yaw) * yaw;
  W(1) = -v * std::cos(yaw) * yaw;
}

}  // namespace mpc_controller 