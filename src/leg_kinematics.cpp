#include "srbd_mpc/leg_kinematics.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"

#include <stdexcept>
#include <iostream>


namespace srbd_mpc {

LegKinematics::LegKinematics(const std::string& urdf, 
                             const std::vector<std::string>& feet) 
  : model_(),
    data_(),
    feet_(),
    fk_(4, Vector3d::Zero()),
    fk_skew_(4, Matrix3d::Zero()) {
  try {
    if (feet.size() != 4) {
      throw std::out_of_range("Invalid argument: feet.size() must be 4!");
    }
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
  pinocchio::urdf::buildModel(urdf, 
                              pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  for (const auto& e : feet) {
    feet_.push_back(model_.getFrameId(e));
  }
}


void LegKinematics::update(const Vector19d& q) {
  pinocchio::framesForwardKinematics(model_, data_, q);
  for (int i=0; i<4; ++i) {
    fk_[i] = data_.oMf[feet_[i]].translation() - data_.com[0];
    pinocchio::skew(fk_[i], fk_skew_[i]);
  }
}

} // namespace srbd_mpc