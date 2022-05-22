#ifndef SRBD_MPC_COST_FUNCTION_HPP_
#define SRBD_MPC_COST_FUNCTION_HPP_

#include <vector>

#include "srbd_mpc/types.hpp"
#include "srbd_mpc/qp_data.hpp"


namespace srbd_mpc {

class CostFunction {
public:
  CostFunction(const Matrix12d& Qxx, const Matrix3d& Quu);

  CostFunction() = default;

  ~CostFunction() = default;

  void setRef(const aligned_vector<Vector12d>& x_ref);

  void initQP(QPData& qp_data);

  void setQP(QPData& qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Matrix12d Qxx_, Quu_;
  aligned_vector<Vector12d> x_ref_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_COST_FUNCTION_HPP_