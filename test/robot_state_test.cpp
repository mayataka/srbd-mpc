#include <vector>
#include <string>

#include <gtest/gtest.h>

#include "srbd_mpc/robot_state.hpp"


namespace srbd_mpc {


class RobotStateTest : public ::testing::Test {
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


TEST_F(RobotStateTest, testRobotState) {
  auto robot = RobotState(urdf, feet);
}

} // namespace srbd_mpc


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}