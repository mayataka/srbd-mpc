#ifndef SRBD_MPC_COST_FUNCTION_HPP_
#define SRBD_MPC_COST_FUNCTION_HPP_

#include <vector>

#include "srbd_mpc/types.hpp"
#include "srbd_mpc/contact_schedule.hpp"
#include "srbd_mpc/qp.hpp"


namespace srbd_mpc {

class CostFunction {
public:
  CostFunction(const Matrix12d& Qxx, const Matrix3d& Quu);

  CostFunction();

  ~CostFunction() = default;

  void setRef(const aligned_vector<Vector12d>& x_ref);

  void update(const ContactSchedule& contact_schedule, i
              pipm::ocp_qp_dim qp_dim, hpipm::ocp_qp qp);
  );

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Matrix12d Qxx_, Quu_;
  Vector12d lx_;
  aligned_vector<Vector12d> x_ref_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_COST_FUNCTION_HPP_