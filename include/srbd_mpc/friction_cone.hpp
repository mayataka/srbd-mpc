#ifndef SRBD_MPC_FRICTION_CONE_HPP_
#define SRBD_MPC_FRICTION_CONE_HPP_

#include <limits>

#include "srbd_mpc/types.hpp"
#include "srbd_mpc/qp_data.hpp"

namespace srbd_mpc {

class FrictionCone {
public:
  FrictionCone(const double mu, const double fzmin=0.0, 
               const double fzmax=std::numeric_limits<double>::infinity());

  FrictionCone();

  ~FrictionCone() = default;

  void setQP(QPData& qp_data) const;

private:
  double mu_, fzmin_, fzmax_;
  MatrixXd cone_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_FRICTION_CONE_HPP_