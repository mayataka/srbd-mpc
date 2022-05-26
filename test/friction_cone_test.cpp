#include <vector>

#include <gtest/gtest.h>

#include "srbd_mpc/friction_cone.hpp"


namespace srbd_mpc {


class FrictionConeTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    mu = 0.6;
    fzmin = 0.1;
    fzmax = std::numeric_limits<double>::infinity();
  }

  virtual void TearDown() {
  }

  double mu, fzmin, fzmax;
};


TEST_F(FrictionConeTest, testFrictionCone) {
  FrictionCone friction_cone(mu ,fzmin, fzmax);
}

} // namespace srbd_mpc


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}