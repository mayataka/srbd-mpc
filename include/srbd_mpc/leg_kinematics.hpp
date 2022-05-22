#ifndef SRBD_MPC_LEG_KINEMATICS_HPP_ 
#define SRBD_MPC_LEG_KINEMATICS_HPP_

#include <string>
#include <vector>
#include <cassert>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "srbd_mpc/types.hpp"


namespace srbd_mpc {

class LegKinematics {
public:
  LegKinematics(const std::string& urdf, const std::vector<std::string>& feet);

  void update(const Vector19d& q);

  const Vector3d& get(const int i) const {
    assert(i >= 0);
    assert(i < 4);
    return fk_[i];
  }

  const Matrix3d& getSkew(const int i) const {
    assert(i >= 0);
    assert(i < 4);
    return fk_skew_[i];
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<int> feet_;
  aligned_vector<Vector3d> fk_;
  aligned_vector<Matrix3d> fk_skew_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_LEG_KINEMATICS_HPP_