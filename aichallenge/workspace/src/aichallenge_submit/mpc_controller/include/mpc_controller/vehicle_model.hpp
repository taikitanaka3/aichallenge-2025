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

#ifndef MPC_CONTROLLER__VEHICLE_MODEL_HPP_
#define MPC_CONTROLLER__VEHICLE_MODEL_HPP_

#include <Eigen/Core>

namespace mpc_controller
{

class LinearVehicleModel
{
public:
  LinearVehicleModel(const double wheel_base, const double steer_lim, const double steer_rate_lim);
  ~LinearVehicleModel() = default;

  // 状態ベクトルを更新
  void updateState(
    const Eigen::VectorXd & x, const double dt, const double steer, const double accel,
    Eigen::VectorXd & next_x);

  // 状態空間モデルの行列を計算
  void calculateStateSpaceMatrix(
    const Eigen::VectorXd & x, const double steer, Eigen::MatrixXd & A, Eigen::MatrixXd & B,
    Eigen::MatrixXd & C, Eigen::MatrixXd & W);

  // 状態ベクトルの次元を取得
  int getDimX() const { return dim_x_; }
  // 入力ベクトルの次元を取得
  int getDimU() const { return dim_u_; }

private:
  // 車両パラメータ
  double wheel_base_;      // ホイールベース [m]
  double steer_lim_;       // ステアリング角制限 [rad]
  double steer_rate_lim_;  // ステアリング角速度制限 [rad/s]

  // 状態空間モデルの次元
  static constexpr int dim_x_ = 4;  // 状態ベクトルの次元
  static constexpr int dim_u_ = 2;  // 入力ベクトルの次元 (ステアリング角, 加速度)
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__VEHICLE_MODEL_HPP_ 