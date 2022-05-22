#include "srbd_mpc/single_rigid_body.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"


namespace srbd_mpc {

SingleRigidBody::SingleRigidBody() 
  model_(),
  data_() {
  pinocchio::Model model_;
  pinocchio::Data data_;
  const auto joint_index = model_.addJoint(model_.getJointId("universe"), 
                                           pinocchio::JointModelFreeFlyer(),
                                           SE3::Identity(), 
                                           "floating_base_joint");
  model_.appendBodyToJoint(joint_index, pinocchio::Inertia::Zero(), 
                           SE3::Identity());
  model_.addBodyFrame("floating_base", joint_index);
  model_.lowerPositionLimit.template segment<4>(3).fill(-1.);
  model_.upperPositionLimit.template segment<4>(3).fill(1.);
}



void SingleRigidBody::integrate(const Vector6d& dq, Vector7d& q) const {
  const Vector7d q_tmp = q;
  pinocchio::integrate(model_, q_tmp, dq, q);
}


void SingleRigidBody::integrate(const Vector7d& q, const Vector6d& dq, 
                                Vector7d& q_next) const {
  pinocchio::integrate(model_, q, dq, q_next);
}


void SingleRigidBody::difference(const Vector7d& qf, const Vector7d& q0, 
                                 Vector6d& qdiff) const {
  pinocchio::difference(model_, q0, qf, qdiff);
}


void SingleRigidBody::dDifference_dqf(const Vector7d& qf, const Vector7d& q0, 
                                      Matrix6d& Jdiff) const {
  pinocchio::dDifference(model_, q0, qf, Jdiff, pinocchio::ARG1);
}


void SingleRigidBody::dDifference_dq0(const Vector7d& qf, const Vector7d& q0, 
                                      Matrix6d& Jdiff) const {
  pinocchio::dDifference(model_, q0, qf, Jdiff, pinocchio::ARG0);
}

} // namespace srbd_mpc
