#include "srbd_mpc/friction_cone.hpp"

#include <stdexcept>
#include <cmath>
#include <iostream>
#include <cmath>


namespace srbd_mpc {

FrictionCone::FrictionCone(const double mu, const double fzmin, const double fzmax)
  : mu_(mu),
    fzmin_(fzmin),
    fzmax_(fzmax),
    cone_(Matrix43d::Zero()) {
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
  cone_ <<  1.0,  0.0, -(mu/std::sqrt(2)),
           -1.0,  0.0, -(mu/std::sqrt(2)),
            0.0,  1.0, -(mu/std::sqrt(2)),
            0.0, -1.0, -(mu/std::sqrt(2));
}


FrictionCone::FrictionCone()
  : mu_(),
    fzmin_(),
    fzmax_(),
    cone_(Matrix43d::Zero()) {
}


void FrictionCone::setQP(QPData& qp_data) {
  for (int i=0; i<qp_data.dim.N; ++i) {
    for (int j=0; j<qp_data.dim.nbu[i]; ++j) {
      qp_data.qp.idxbu[i][j] = 3*j + 2;
    }
    qp_data.qp.lbu[i].fill(fzmin_);
    qp_data.qp.ubu[i].fill(fzmax_);
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.C[i].setZero();
    qp_data.qp.D[i].setZero();
    for (int j=0; j<qp_data.dim.nbu[i]; ++j) {
      qp_data.qp.D[i].template block<4, 3>(4*j, 3*j) = cone_;
    }
    qp_data.qp.lg[i].setZero();
    qp_data.qp.ug[i].setZero();
    qp_data.qp.lg_mask[i].fill(1.0);
  }
}

} // namespace srbd_mpc