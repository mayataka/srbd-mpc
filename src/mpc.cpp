#include "srbd_mpc/mpc.hpp"

#include <cassert>


namespace srbd_mpc {

MPC::MPC(const StateEquation& state_equation, const CostFunction& cost_function, 
         const FrictionCone& friction_cone)
  : state_equation_(state_equation),
    cost_function_(cost_function),
    friction_cone_(friction_cone),
    qp_data_(),
    qp_solver_() {
}


void MPC::init(const ContactSchedule& contact_schedule) {
  qp_data_.init(contact_schedule);
  qp_solver_.init(qp_data_);
  state_equation_.initQP(qp_data_);
  cost_function_.initQP(qp_data_);
  assert(qp_data_.checkSize());
  mpc_solution_.init(contact_schedule);
}


void MPC::solve(const ContactSchedule& contact_schedule,
                const GaitCommand& gait_command, const RobotState& robot_state) {
  qp_data_.resize(contact_schedule);
  state_equation_.setQP(contact_schedule, robot_state, qp_data_);
  cost_function_.setQP(contact_schedule, robot_state, gait_command, qp_data_);
  friction_cone_.setQP(qp_data_);
  qp_solver_.solve(qp_data_);
  assert(qp_data_.checkSize());
  mpc_solution_.update(contact_schedule, robot_state, qp_data_);
}

} // namespace srbd_mpc