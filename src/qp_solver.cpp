#include "srbd_mpc/qp_solver.hpp"


namespace srbd_mpc {

void QPSolver::init(QPData& qp_data) {
  settings.createHpipmData(qp_data.dim);
  solver_.createHpipmData(qp_data.dim, settings);
}


void QPSolver::solve(QPData& qp_data) {
  settings.createHpipmData(qp_data.dim);
  solver_.createHpipmData(qp_data.dim, settings);
  solver_.solve(qp_data.qp, qp_data.qp_solution, settings);
}

} // namespace srbd_mpc