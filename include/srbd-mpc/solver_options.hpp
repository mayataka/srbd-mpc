#ifndef SRBD_MPC_SOLVER_OPTIONS_HPP_
#define SRBD_MPC_SOLVER_OPTIONS_HPP_

#include <vector>

#include "srbd-mpc/types.hpp"
#include "srbd-mpc/contact_schedule.hpp"
#include "srbd-mpc/robot_state.hpp"
#include "srbd-mpc/qp_data.hpp"


namespace srbdmpc {

struct SolverOptions {
  double mu0 = 1.0e+02; // intial barrier parameter

  double tol_stat = 1.0e-04; // convergence criteria

  double tol_eq = 1.0e-04; // convergence criteria

  double tol_ineq = 1.0e-04; // convergence criteria

  double tol_comp = 1.0e-04; // convergence criteria

  double reg_prim = 1.0e-12; // reg

  int warm_start = 0; // use warm start or not

  int pred_corr = 1; // use correction step

  int ric_alg = 1; // use square-root Riccati or not 

  int split_step = 0; //  use different step for primal and dual variables or not
};

} // namespace srbdmpc

#endif // SRBD_MPC_SOLVER_OPTIONS_HPP_