#ifndef SRBD_MPC_STATE_EQUATION_HPP_
#define SRBD_MPC_STATE_EQUATION_HPP_

#include <vector>

#include "srbd_mpc/types.hpp"
#include "srbd_mpc/leg_kinematics.hpp"
#include "srbd_mpc/contact_schedule.hpp"
#include "srbd_mpc/qp_data.hpp"


namespace srbd_mpc {

class StateEquation {
public:
  StateEquation(const double dt, const double m=22.5, 
                const Matrix3d& I=(Matrix3d() << 0.050874, 0., 0., 
                                                 0., 0.64036, 0., 
                                                 0., 0., 0.6565).finished(), 
                const Vector3d& g=(Vector3d() << 0., 0., -9.81).finished());

  void initQP(QPData& qp_data);

  void setQP(const double yaw_angle, const LegKinematics& leg_kinematics,  
             const ContactSchedule& contact_schedule, QPData& qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  int N_;
  double dt_, m_;
  Vector3d g_;
  Matrix3d R_, I_local_, I_global_, I_global_inv_;
  aligned_vector<Matrix3d> I_inv_r_skew_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_STATE_EQUATION_HPP_