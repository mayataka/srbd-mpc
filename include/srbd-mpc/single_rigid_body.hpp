#ifndef SRBD_MPC_SINGLE_RIGID_BODY_HPP_
#define SRBD_MPC_SINGLE_RIGID_BODY_HPP_

#include <vector>

#include "srbd-mpc/types.hpp"
#include "srbd-mpc/contact_schedule.hpp"
#include "srbd-mpc/robot_state.hpp"
#include "srbd-mpc/qp_data.hpp"


namespace srbdmpc {

class SingleRigidBody {
public:
  SingleRigidBody();

  ~SingleRigidBody() = default;

  void integrate(const Vector6d& dq, Vector7d& q) const;

  void integrate(const Vector7d& q, const Vector6d& dq, Vector7d& q_next) const;

  void difference(const Vector7d& qf, const Vector7d& q0, Vector6d& qdiff) const;

  void dDifference_dqf(const Vector7d& qf, const Vector7d& q0, 
                       Matrix6d& Jdiff) const;

  void dDifference_dq0(const Vector7d& qf, const Vector7d& q0, 
                       Matrix6d& Jdiff) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
};

} // namespace srbdmpc

#endif // SRBD_MPC_SINGLE_RIGID_BODY_HPP_