#ifndef SRBD_MPC_GAIT_COMMAND_HPP_
#define SRBD_MPC_GAIT_COMMAND_HPP_

#include "srbd-mpc/types.hpp"


namespace srbdmpc {

struct GaitCommand {
public:
  GaitCommand() = default;

  ~GaitCommand() = default;

  Vector3d vcom;

  double yaw_rate;
};

} // namespace srbdmpc

#endif // SRBD_MPC_GAIT_COMMAND_HPP_