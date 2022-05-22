#include "srbd_mpc/state_equation.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>
#include <cassert>

namespace srbd_mpc {

StateEquation::StateEquation(const double dt, const double m, const Matrix3d& I, 
                             const Vector3d& g)
  : m_(m),
    g_(g),
    R_(Matrix3d::Identity()),
    I_local_(I),
    I_global_(I),
    I_global_inv_(I_global_.inverse()),
    I_inv_r_skew_(4, Matrix3d::Zero()) {
  try {
    if (dt <= 0.0) {
      throw std::out_of_range("Invalid argument: dt must be positive!");
    }
    if (m <= 0.0) {
      throw std::out_of_range("Invalid argument: m must be positive!");
    }
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
}


void StateEquation::initQP(QPData& qp_data) const {
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.A[i].setZero();
    qp_data.qp.A[i].template block<3, 3>(3, 9) = dt_ * Matrix3d::Identity();
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.B[i].setZero();
    qp_data.qp.B[i].template block<3, 3>(9, i*3) = (dt_/m_) * Matrix3d::Identity();
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.b[i].setZero();
    qp_data.qp.b[i].template tail<3>() = g_;
  }
}


void StateEquation::setQP(const ContactSchedule& contact_schedule, 
                          const RobotState& robot_state, QPData& qp_data) {
  // rotation matrix
  R_ = robot_state.R();
  // global inertia matrix
  I_global_inv_.noalias() = R_.transpose() * I_local_;
  I_global_.noalias() = I_global_inv_ * R_;
  I_global_inv_ = I_global_.inverse();
  // dynamics w.r.t. control input
  for (int i=0; i<4; ++i) {
    I_inv_r_skew_[i].noalias() 
        = dt_ * I_global_inv_ * robot_state.getLegKinematicsSkew(i);
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.A[i].template block<3, 3>(0, 6) = dt_ * R_;
    int nu = 0;
    for (int j=0; j<4;++j) {
      if (contact_schedule.isContactActive(i)[j]) {
        qp_data.qp.B[i].template block<3, 3>(6, nu) = I_inv_r_skew_[j];
        nu += 3;
      } 
    }
  }
}

} // namespace srbd_mpc