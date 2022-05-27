#include "srbd-mpc/cost_function.hpp"


namespace srbdmpc {

CostFunction::CostFunction(const double dt, const Matrix6d& Qqq, 
                           const Matrix6d& Qvv, const Matrix3d& Quu)
  : dt_(dt),
    Qqq_(Qqq),
    Qvv_(Qvv),
    Quu_(Matrix12d::Zero()),
    base_pose_(Vector7d::Zero()),
    base_pose_ref_(),
    single_rigid_body_(),
    qdiff_(Vector6d::Zero()),
    Jqdiff_(Matrix6d::Zero()),
    JtQqq_(Matrix6d::Zero()) {
  for (int i=0; i<4; ++i) {
    Quu_.template block<3, 3>(3*i, 3*i) = Quu;
  }
}


void CostFunction::initQP(QPData& qp_data) {
  for (int i=0; i<qp_data.dim.N+1; ++i) {
    qp_data.qp.Q[i].template topLeftCorner<6, 6>() = Qqq_;
    qp_data.qp.Q[i].template bottomRightCorner<6, 6>() = Qvv_;
  }
  base_pose_ref_ = aligned_vector<Vector7d>(qp_data.dim.N+1, Vector7d::Zero());
}



void CostFunction::setQP(const ContactSchedule& contact_schedule, 
                         const RobotState& robot_state, 
                         const GaitCommand& gait_command, QPData& qp_data) {
  v_command_.template head<3>() = gait_command.vcom;
  v_command_.template tail<3>() << 0., 0., gait_command.yaw_rate;
  dq_command_ = dt_ * v_command_;
  base_pose_.template head<3>() = robot_state.com();
  base_pose_.template tail<4>() = robot_state.quat().coeffs();
  // The reference rotation whose pitch and roll angles are 0.
  const double cref = robot_state.R().coeff(0, 0);
  const double sref = robot_state.R().coeff(1, 0);
  const double l2normref = std::sqrt(cref*cref + sref*sref);
  Eigen::Matrix3d R_ref; 
  R_ref << cref/l2normref, -sref/l2normref,  0.,
           sref/l2normref,  cref/l2normref,  0.,
                        0.,              0., 1.;
  base_pose_ref_[0].template head<3>() = robot_state.com();
  base_pose_ref_[0].template tail<4>() = Quaterniond(R_ref).coeffs();
  for (int i=0; i<qp_data.dim.N; ++i) {
    single_rigid_body_.integrate(base_pose_ref_[i], dq_command_, base_pose_ref_[i+1]);
  }
  // qdiff(q, qref)^T Q qdiff(q, qref)
  // -> linearize: (qdiff(q, qref) + Jqdiff_dqf(q, qref) dq) ^T Q (qdiff(q, qref) + Jqdiff_dqf(q, qref) dq)
  for (int i=0; i<qp_data.dim.N+1; ++i) {
    single_rigid_body_.difference(base_pose_, base_pose_ref_[i], qdiff_);
    single_rigid_body_.dDifference_dqf(base_pose_, base_pose_ref_[i], Jqdiff_);
    JtQqq_.noalias() = Jqdiff_.transpose() * Qqq_;
    qp_data.qp.Q[i].template topLeftCorner<6, 6>().noalias() = JtQqq_ * Jqdiff_;
    qp_data.qp.q[i].template head<6>().noalias() = JtQqq_ * qdiff_;
    qp_data.qp.q[i].template tail<6>().noalias() = Qvv_ * (robot_state.twist() - v_command_);
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.S[i].setZero();
    qp_data.qp.R[i] = Quu_.topLeftCorner(qp_data.dim.nu[i], qp_data.dim.nu[i]);
    qp_data.qp.r[i].setZero();
  }
}

} // namespace srbdmpc