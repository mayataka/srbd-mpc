#include "srbd_mpc/cost_function.hpp"


namespace srbd_mpc {

CostFunction::CostFunction(const Matrix12d& Qxx, const Matrix3d& Quu)
  : Qxx_(Qxx),
    Quu_(Matrix12d::Zero()),
    x_ref_() {
  for (int i=0; i<4; ++i) {
    Quu_.template block<3, 3>(3*i, 3*i) = Quu;
  }
}


void CostFunction::setRef(const aligned_vector<Vector12d>& x_ref) {
  x_ref_ = x_ref;
}


void CostFunction::initQP(QPData& qp_data) {
  for (int i=0; i<qp_data.dim.N+1; ++i) {
    qp_data.qp.Q[i] = Qxx_;
  }
}


void CostFunction::setQP(QPData& qp_data) {
  for (int i=0; i<qp_data.dim.N+1; ++i) {
    qp_data.qp.q[i].noalias() = - Qxx_ * x_ref_[i];
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.S[i].setZero();
    qp_data.qp.R[i] = Quu_.topLeftCorner(qp_data.dim.nu[i], qp_data.dim.nu[i]);
    qp_data.qp.r[i].setZero();
  }
}

} // namespace srbd_mpc