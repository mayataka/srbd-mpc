#ifndef SRBD_MPC_QP_DATA_HPP_
#define SRBD_MPC_QP_DATA_HPP_

#include <vector>
#include <string>

#include "hpipm-cpp/hpipm-cpp.hpp"
#include "srbd_mpc/contact_schedule.hpp"


namespace srbd_mpc {

struct QPData {
public:
  QPData() = default;

  ~QPData() = default;

  void init(const ContactSchedule& contact_schedule);

  void resize(const ContactSchedule& contact_schedule);

  bool checkSize() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  hpipm::OcpQpDim dim;
  hpipm::OcpQp qp;
  hpipm::OcpQpSolution qp_solution;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_QP_DATA_HPP_