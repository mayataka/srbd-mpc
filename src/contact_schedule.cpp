#include "srbd_mpc/contact_schedule.hpp" 

#include <stdexcept> 
#include <iostream>
#include <cmath>


namespace srbd_mpc {

ContactSchedule::ContactSchedule(const double T, const int N) 
  : T_(T), 
    dt_(T/N),
    N_(N), 
    t_({0.}),
    is_contact_active_({std::vector<bool>({true, true, true, true})}),
    phase_(N+1, 0) {
  try {
    if (T <= 0.0) {
      throw std::out_of_range("Invalid argument: T must be positive!");
    }
    if (N <= 0) {
      throw std::out_of_range("Invalid argument: N must be positive!");
    }
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
}


void ContactSchedule::reset(const double t, 
                            const std::vector<bool>& is_contact_active) {
  assert(is_contact_active.size() == 4);
  is_contact_active_.clear();
  is_contact_active_.push_back(is_contact_active);
  t_.clear();
  t_.push_back(t);
  std::fill(phase_.begin(), phase_.end(), 0);
}


void ContactSchedule::push_back(const double t, 
                                const std::vector<bool>& is_contact_active) {
  assert(is_contact_active.size() == 4);
  if (t > t_.back() + dt_ && t < t_.front() + T_) {
    is_contact_active_.clear();
    is_contact_active_.push_back(is_contact_active);
    t_.push_back(t);
    const int stage_begin = std::floor(t-t_.front() / dt_);
    const int next_phase = phase_.back() + 1;
    std::fill(phase_.begin()+stage_begin, phase_.end(), next_phase);
  }
}

} // namespace srbd_mpc