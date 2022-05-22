#include "srbd_mpc/qp_data.hpp"


namespace srbd_mpc {

void QPData::init(const ContactSchedule& contact_schedule) {
  // Here, we allocate memory as the possible maximum size.
  dim.resize(contact_schedule.N());
  const int nx = 12;
  const int nu = 12; // possible max size
  hpipm::fill_vector(dim.nx, nx);
  hpipm::fill_vector(dim.nu, nu);
  hpipm::fill_vector(dim.nbx, 0);
  dim.nbx.front() = nx;
  hpipm::fill_vector(dim.nbu, 4); // fz_min < fz < fz_max for 4 legs
  hpipm::fill_vector(dim.ng, 16); // -inf < cone * [fx, fy, fz] < 0 for 4 legs
  dim.ng.back() = 0;
  const auto dim_err_msg = dim.checkSize();
  if (!dim_err_msg.empty()) {
    for (const auto& e : dim_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  dim.createHpipmData();

  qp.resize(dim);
  qp.lg_mask.resize(dim.N+1);
  hpipm::fill_vector(qp.lg_mask, Eigen::VectorXd::Zero(4));
  const auto qp_err_msg = qp.checkSize(dim);
  if (!qp_err_msg.empty()) {
    for (const auto& e : qp_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp.createHpipmData(dim);

  qp_solution.resize(dim);
  const auto sol_err_msg = qp_solution.checkSize(dim);
  if (!sol_err_msg.empty()) {
    for (const auto& e : sol_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp_solution.createHpipmData(dim);
}


void QPData::resize(const ContactSchedule& contact_schedule) {
  for (int i=0; i<dim.N; ++i) {
    dim.nu[i] = 3 * contact_schedule.numActiveContacts(i);
  }
  for (int i=0; i<dim.N; ++i) {
    dim.nbu[i] = contact_schedule.numActiveContacts(i);
  }
  for (int i=0; i<dim.N; ++i) {
    dim.ng[i] = 4 * contact_schedule.numActiveContacts(i);
  }
  const auto dim_err_msg = dim.checkSize();
  if (!dim_err_msg.empty()) {
    for (const auto& e : dim_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  dim.createHpipmData();

  // dynamics
  for (int i=0; i<dim.N; ++i) {
    qp.B[i].resize(dim.nx[i], dim.nu[i]);
  }
  // cost
  for (int i=0; i<dim.N; ++i) {
    qp.S[i].resize(dim.nu[i], dim.nx[i]);
    qp.R[i].resize(dim.nu[i], dim.nu[i]);
    qp.r[i].resize(dim.nu[i]);
  }
  // constraints
  for (int i=0; i<dim.N; ++i) {
    qp.lbu[i].resize(dim.nbu[i]);
    qp.ubu[i].resize(dim.nbu[i]);
  }
  for (int i=0; i<dim.N; ++i) {
    qp.C[i].resize(dim.ng[i], dim.nx[i]);
    qp.D[i].resize(dim.ng[i], dim.nu[i]);
    qp.lg[i].resize(dim.ng[i]);
    qp.ug[i].resize(dim.ng[i]);
    qp.lg_mask[i].resize(dim.ng[i]);
  }
  const auto qp_err_msg = qp.checkSize(dim);
  if (!qp_err_msg.empty()) {
    for (const auto& e : qp_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp.createHpipmData(dim);

  for (int i=0; i<dim.N; ++i) {
    qp_solution.u[i].resize(dim.nu[i]);
  }
  const auto sol_err_msg = qp_solution.checkSize(dim);
  if (!sol_err_msg.empty()) {
    for (const auto& e : sol_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp_solution.createHpipmData(dim);
}

} // namespace srbd_mpc
