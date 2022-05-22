#ifndef SRBD_MPC_QP_SOLVER_HPP_
#define SRBD_MPC_QP_SOLVER_HPP_

#include "hpipm-cpp/hpipm-cpp.hpp"
#include "srbd_mpc/qp_data.hpp"


namespace srbd_mpc {

class QPSolver {
public:
  QPSolver() = default;

  ~QPSolver() = default;

  void init(QPData& qp_data);

  void solve(QPData& qp_data);

  hpipm::OcpQpIpmSolverSettings settings;

private:
  hpipm::OcpQpIpmSolver solver_;

};

} // namespace srbd_mpc

#endif // SRBD_MPC_QP_DATA_HPP_