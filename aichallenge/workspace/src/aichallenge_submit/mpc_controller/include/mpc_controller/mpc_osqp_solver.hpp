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

#ifndef MPC_OSQP_SOLVER_HPP_
#define MPC_OSQP_SOLVER_HPP_

#include <Eigen/Core>
#include <vector>

// OSQP C API
#include <osqp.h>

namespace mpc_controller {

/**
 * @brief OSQPを使ったMPCソルバの最小クラス
 *
 * 使い方例：
 *   mpc_controller::MPCOSQPSolver solver(horizon, state_dim, input_dim);
 *   solver.setWeight(Q, R);
 *   solver.setModel(A, B);
 *   solver.setConstraint(u_min, u_max);
 *   Eigen::VectorXd u = solver.solve(x0, x_ref_seq);
 */
class MPCOSQPSolver {
public:
  MPCOSQPSolver(int horizon, int state_dim, int input_dim);
  ~MPCOSQPSolver();

  void setWeight(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
  void setModel(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
  void setConstraint(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max);

  // x0: 現在状態, x_ref_seq: 目標状態列
  // 戻り値: 最初の制御入力（input_dim次元）
  Eigen::VectorXd solve(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& x_ref_seq);

private:
  int N_; // ホライズン長
  int nx_; // 状態次元
  int nu_; // 入力次元

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;

  // OSQPワークスペース
  OSQPWorkspace* work_ = nullptr;
  OSQPSettings* settings_ = nullptr;
  OSQPData* data_ = nullptr;

  void cleanup();
};

} // namespace mpc_controller

#endif // MPC_OSQP_SOLVER_HPP_ 