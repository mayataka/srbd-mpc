#ifndef SRBD_MPC_QP_SOLVER_HPP_
#define SRBD_MPC_QP_SOLVER_HPP_

#include "hpipm-cpp/hpipm-cpp.hpp"
#include "srbd-mpc/qp_data.hpp"


namespace srbdmpc {

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

} // namespace srbdmpc

#endif // SRBD_MPC_QP_DATA_HPP_