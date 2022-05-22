#ifndef SRBD_MPC_TYPES_HPP_
#define SRBD_MPC_TYPES_HPP_

#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/StdVector"

namespace srbd_mpc {

using Matrix3d   = Eigen::Matrix<double, 3, 3>;
using Matrix6d   = Eigen::Matrix<double, 6, 6>;
using Matrix9d   = Eigen::Matrix<double, 9, 9>;
using Matrix53d  = Eigen::Matrix<double, 5, 3>;
using Matrix43d  = Eigen::Matrix<double, 4, 3>;
using Matrix12d  = Eigen::Matrix<double, 12, 12>;
using Matrix12Xd = Eigen::Matrix<double, 12, Eigen::Dynamic>;
using MatrixXd   = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Vector3d   = Eigen::Matrix<double, 3, 1>;
using Vector5d   = Eigen::Matrix<double, 5, 1>;
using Vector12d  = Eigen::Matrix<double, 12, 1>;
using Vector19d  = Eigen::Matrix<double, 19, 1>;
using VectorXd   = Eigen::VectorXd;

using Eigen::Block;

using Eigen::VectorBlock;

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;


} // namespace srbd_mpc

#endif // SRBD_MPC_TYPES_HPP_