#ifndef SRBD_MPC_MPC_HPP_
#define SRBD_MPC_MPC_HPP_

#include "srbd-mpc/contact_schedule.hpp"
#include "srbd-mpc/gait_command.hpp"
#include "srbd-mpc/robot_state.hpp"
#include "srbd-mpc/state_equation.hpp"
#include "srbd-mpc/cost_function.hpp"
#include "srbd-mpc/friction_cone.hpp"
#include "srbd-mpc/qp_data.hpp"
#include "srbd-mpc/qp_solver.hpp"
#include "srbd-mpc/mpc_solution.hpp"
#include "srbd-mpc/solver_options.hpp"


namespace srbdmpc {

class MPC {
public:
  MPC(const StateEquation& state_equation, const CostFunction& cost_function, 
      const FrictionCone& friction_cone);

  MPC() = default;

  ~MPC() = default;

  void setOptions(const SolverOptions& solver_options);

  void init(const ContactSchedule& contact_schedule);

  void solve(const RobotState& robot_state, 
             const ContactSchedule& contact_schedule,
             const GaitCommand& gait_command);

  const MPCSolution& getSolution() const { return mpc_solution_; }

private:
  StateEquation state_equation_;
  CostFunction cost_function_;
  FrictionCone friction_cone_;
  QPData qp_data_;
  QPSolver qp_solver_;
  MPCSolution mpc_solution_;
};

} // namespace srbdmpc

#endif // SRBD_MPC_MPC_HPP_