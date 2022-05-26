#include <vector>
#include <string>

#include <gtest/gtest.h>

#include "srbd_mpc/mpc.hpp"


namespace srbd_mpc {


class MPCTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    urdf = "urdf/a1.urdf";
    feet = {"FL_foot", "RL_foot", "FR_foot", "RR_foot"};
    dt = 0.01;
    mu = 0.6;
    fzmin = 0.0;
    fzmax = std::numeric_limits<double>::infinity();
    Qqq = Eigen::VectorXd::Constant(6, 10.0).asDiagonal();
    Qvv = Eigen::VectorXd::Constant(6, 0.1).asDiagonal();
    Quu = Eigen::VectorXd::Constant(3, 0.001).asDiagonal();

    T = 1.0;
    N = 20;
  }

  virtual void TearDown() {
  }

  std::string urdf;
  std::vector<std::string> feet;
  double dt, mu, fzmin, fzmax;
  Eigen::MatrixXd Qqq, Qvv, Quu;

  double T;
  int N;
};


TEST_F(MPCTest, testMPC) {
  auto state_equation = StateEquation(dt);
  auto cost_function = CostFunction(dt, Qqq, Qvv, Quu);
  auto friction_cone = FrictionCone(mu, fzmin, fzmax);
  // auto mpc = MPC(state_equation, cost_function, friction_cone);
  MPC mpc(state_equation, cost_function, friction_cone);

  auto contact_schedule = ContactSchedule(T, N);
  const double t0 = 0.0;
  const double t1 = 0.3;
  const double t2 = 0.55;
  const double t3 = 0.7;
  contact_schedule.reset(t0, {true, true, true, true});
  contact_schedule.push_back(t1, {false, true, false, true});
  contact_schedule.push_back(t2, {false, true, true, true});
  contact_schedule.push_back(t3, {true, false, false, false});
  mpc.init(contact_schedule);

  auto robot_state = RobotState(urdf, feet);
  Eigen::VectorXd q = Eigen::VectorXd::Random(19);
  q.segment<4>(3) = Eigen::Quaternion<double>::UnitRandom().coeffs();
  Eigen::VectorXd v = Eigen::VectorXd::Random(18);
  robot_state.update(q, v);
  auto gait_command = GaitCommand();
  gait_command.vcom = Eigen::VectorXd::Random(3);
  gait_command.yaw_rate = Eigen::VectorXd::Random(1)[0];
  mpc.solve(contact_schedule, gait_command, robot_state);
}

} // namespace srbd_mpc


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}