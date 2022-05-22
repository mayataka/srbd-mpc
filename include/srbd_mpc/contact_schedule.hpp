#ifndef SRBD_MPC_CONTACT_SCHEDULE_HPP_
#define SRBD_MPC_CONTACT_SCHEDULE_HPP_

#include <vector>
#include <deque>
#include <cassert>


namespace srbd_mpc {

class ContactSchedule {
public:
  ContactSchedule(const double T, const int N);

  ContactSchedule() = default;

  ~ContactSchedule() = default;

  void reset(const double t, const std::vector<bool>& is_contact_active);

  void push_back(const double t, const std::vector<bool>& is_contact_active);

  int phase(const int stage) const {
    assert(stage >= 0);
    assert(stage <= N_);
    return phase_[stage];
  }

  int numActiveContacts(const int stage) const {
    assert(stage >= 0);
    assert(stage <= N_);
    return phase_[stage];
  }

  const std::vector<bool> isContactActive(const int phase) const {
    assert(phase >= 0);
    assert(phase <= N_);
    return is_contact_active_[phase];
  }

  int N() const {
    return N_;
  }

private:
  double T_, dt_;
  int N_;
  std::vector<double> t_;
  std::vector<std::vector<bool>> is_contact_active_;
  std::vector<int> phase_;
};

} // namespace srbd_mpc

#endif // SRBD_MPC_CONTACT_SCHEDULE_HPP_