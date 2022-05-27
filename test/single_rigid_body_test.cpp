#include <vector>
#include <string>

#include <gtest/gtest.h>

#include "srbd-mpc/single_rigid_body.hpp"


namespace srbdmpc {


class SingleRigidBodyTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    urdf = "urdf/a1.urdf";
    feet = {"FL_foot", "RL_foot", "FR_foot", "RR_foot"};
  }

  virtual void TearDown() {
  }

  std::string urdf;
  std::vector<std::string> feet;
};


TEST_F(SingleRigidBodyTest, testSingleRigidBody) {
  auto single_rigid_body = SingleRigidBody();
}

} // namespace srbdmpc


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}