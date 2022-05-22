#ifndef SRBD_MPC_GAIT_COMMAND_HPP_
#define SRBD_MPC_GAIT_COMMAND_HPP_

#include <vector>

#include "srbd_mpc/types.hpp"
#include "srbd_mpc/leg_kinematics.hpp"
#include "srbd_mpc/contact_schedule.hpp"
#include "srbd_mpc/qp_data.hpp"


namespace srbd_mpc {

struct GaitCommand {
public:
  GaitCommand() = default;

  ~GaitCommand() = default;

  Vector3d vcom;

  double yaw_rate;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_GAIT_COMMAND_HPP_