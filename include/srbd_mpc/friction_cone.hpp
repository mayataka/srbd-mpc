#ifndef SRBD_MPC_FRICTION_CONE_HPP_
#define SRBD_MPC_FRICTION_CONE_HPP_

#include "srbd_mpc/types.hpp"
#include "srbd_mpc/qp_data.hpp"

namespace srbd_mpc {

class FrictionCone {
public:
  FrictionCone(const double mu, const double fzmin=0.0, const double fzmax=1.0e08);

  FrictionCone();

  ~FrictionCone() = default;

  void setQP(QPData& qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  double mu_, fzmin_, fzmax_;
  Matrix43d cone_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_FRICTION_CONE_HPP_