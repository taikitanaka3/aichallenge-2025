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

#include "mpc_controller/mpc_osqp_solver.hpp"
#include <iostream>

namespace mpc_controller {

MPCOSQPSolver::MPCOSQPSolver(int horizon, int state_dim, int input_dim)
  : N_(horizon), nx_(state_dim), nu_(input_dim) {}

MPCOSQPSolver::~MPCOSQPSolver() { cleanup(); }

void MPCOSQPSolver::setWeight(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
  Q_ = Q;
  R_ = R;
}

void MPCOSQPSolver::setModel(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
  A_ = A;
  B_ = B;
}

void MPCOSQPSolver::setConstraint(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max) {
  u_min_ = u_min;
  u_max_ = u_max;
}

void MPCOSQPSolver::cleanup() {
  if (work_) osqp_cleanup(work_);
  if (data_) {
    if (data_->A) c_free(data_->A);
    if (data_->P) c_free(data_->P);
    c_free(data_);
  }
  if (settings_) c_free(settings_);
  work_ = nullptr;
  data_ = nullptr;
  settings_ = nullptr;
}

Eigen::VectorXd MPCOSQPSolver::solve(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& x_ref_seq) {
  // Nステップ分のQPを組み立てる
  // 変数: [u_0, u_1, ..., u_{N-1}]
  // 制約: 状態遷移 x_{k+1} = A x_k + B u_k
  // コスト: sum_{k=0}^{N-1} (x_k - x_ref_k)^T Q (x_k - x_ref_k) + u_k^T R u_k

  int n = N_ * nu_; // 全制御入力の次元
  int m = N_ * nx_; // 全状態の次元

  // 状態遷移行列の構築
  Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(m, nx_); // 状態遷移の係数行列
  Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(m, n);   // 入力の係数行列

  // 最初の状態
  Sx.block(0, 0, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);

  // 状態遷移の計算
  for (int i = 0; i < N_; ++i) {
    int row = i * nx_;
    int col = i * nu_;
    
    // 状態遷移の係数
    if (i < N_ - 1) {
      Sx.block((i + 1) * nx_, 0, nx_, nx_) = A_ * Sx.block(row, 0, nx_, nx_);
    }
    
    // 入力の係数
    Su.block(row, col, nx_, nu_) = B_;
    for (int j = 0; j < i; ++j) {
      Su.block(row, j * nu_, nx_, nu_) = A_ * Su.block((i - 1) * nx_, j * nu_, nx_, nu_);
    }
  }

  // コスト関数の構築
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(m, m);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n, n);
  
  for (int i = 0; i < N_; ++i) {
    Q_bar.block(i * nx_, i * nx_, nx_, nx_) = Q_;
    R_bar.block(i * nu_, i * nu_, nu_, nu_) = R_;
  }

  // 目標状態との差分を計算
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(m);
  for (int i = 0; i < std::min(N_, static_cast<int>(x_ref_seq.size())); ++i) {
    x_ref.segment(i * nx_, nx_) = x_ref_seq[i];
  }

  // QPの定式化
  // min 0.5 u^T H u + f^T u
  // s.t. u_min <= u <= u_max
  Eigen::MatrixXd H = Su.transpose() * Q_bar * Su + R_bar;
  Eigen::VectorXd f = Su.transpose() * Q_bar * (Sx * x0 - x_ref);

  // OSQP用データ構造
  int P_nnz = n; // 対角行列のみ
  std::vector<c_float> P_x(n);
  std::vector<c_int> P_i(n);
  std::vector<c_int> P_p(n+1);
  for (int i = 0; i < n; ++i) {
    P_x[i] = H(i,i);
    P_i[i] = i;
    P_p[i] = i;
  }
  P_p[n] = n;

  // 線形項
  std::vector<c_float> q(n);
  for (int i = 0; i < n; ++i) q[i] = f(i);

  // 制約（単純な入力制約のみ）
  std::vector<c_float> l(n), u(n);
  for (int i = 0; i < n; ++i) {
    l[i] = u_min_(i % nu_);
    u[i] = u_max_(i % nu_);
  }

  // 単位行列A
  std::vector<c_float> A_x(n);
  std::vector<c_int> A_i(n);
  std::vector<c_int> A_p(n+1);
  for (int i = 0; i < n; ++i) {
    A_x[i] = 1.0;
    A_i[i] = i;
    A_p[i] = i;
  }
  A_p[n] = n;

  cleanup();
  data_ = (OSQPData*)c_malloc(sizeof(OSQPData));
  data_->n = n;
  data_->m = n;
  data_->P = csc_matrix(n, n, P_nnz, P_x.data(), P_i.data(), P_p.data());
  data_->q = q.data();
  data_->A = csc_matrix(n, n, n, A_x.data(), A_i.data(), A_p.data());
  data_->l = l.data();
  data_->u = u.data();

  settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
  osqp_set_default_settings(settings_);
  settings_->alpha = 1.0;

  osqp_setup(&work_, data_, settings_);
  osqp_solve(work_);

  Eigen::VectorXd u_sol = Eigen::VectorXd::Zero(nu_);
  if (work_->info->status_val == OSQP_SOLVED) {
    // 最初の制御入力のみを返す
    for (int i = 0; i < nu_; ++i) u_sol(i) = work_->solution->x[i];
  } else {
    std::cerr << "[MPCOSQPSolver] OSQP failed: " << work_->info->status << std::endl;
  }
  return u_sol;
}

} // namespace mpc_controller 