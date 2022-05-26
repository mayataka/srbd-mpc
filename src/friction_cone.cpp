#include "srbd_mpc/friction_cone.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>


namespace srbd_mpc {

FrictionCone::FrictionCone(const double mu, const double fzmin, const double fzmax)
  : mu_(mu),
    fzmin_(fzmin),
    fzmax_(fzmax),
    cone_(MatrixXd::Zero(16, 12)) {
  try {
    if (mu <= 0.0) {
      throw std::out_of_range("Invalid argument: mu must be positive!");
    }
    if (fzmin < 0.0) {
      throw std::out_of_range("Invalid argument: fzmin must be non-negative!");
    }
    if (fzmax <= fzmin) {
      throw std::out_of_range("Invalid argument: fzmax must be larger than fzmin!");
    }
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
  MatrixXd cone(4, 3);
  cone <<  1.0,  0.0, -(mu/std::sqrt(2)),
          -1.0,  0.0, -(mu/std::sqrt(2)),
           0.0,  1.0, -(mu/std::sqrt(2)),
           0.0, -1.0, -(mu/std::sqrt(2));
  for (int i=0; i<4; ++i) {
    cone_.block(4*i, 3*i, 4, 3) = cone;
  }
}


FrictionCone::FrictionCone()
  : mu_(),
    fzmin_(),
    fzmax_() {
}


void FrictionCone::setQP(QPData& qp_data) const {
  for (int i=0; i<qp_data.dim.N; ++i) {
    for (int j=0; j<qp_data.dim.nbu[i]; ++j) {
      qp_data.qp.idxbu[i][j] = 3*j + 2;
    }
    qp_data.qp.lbu[i].fill(fzmin_);
    qp_data.qp.ubu[i].fill(fzmax_);
    if (std::isinf(fzmax_)) {
      qp_data.qp.ubu_mask[i].fill(1.0);
    }
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.C[i].setZero();
    const int num_conatcts = qp_data.dim.nbu[i] / 3;
    assert(qp_data.dim.nbu[i] % 3 == 0);
    if (num_conatcts > 0) {
      qp_data.qp.D[i] 
          = cone_.topLeftCorner(qp_data.qp.D[i].rows(), qp_data.qp.D[i].cols());
    }
    qp_data.qp.lg[i].setZero();
    qp_data.qp.ug[i].setZero();
    qp_data.qp.lg_mask[i].fill(1.0);
  }
}

} // namespace srbd_mpc