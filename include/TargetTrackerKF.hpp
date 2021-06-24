
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <sophus/se3.hpp>

namespace Eigen {
typedef Matrix<double, 12, 12> Matrix12d;
typedef Matrix<double, 12, 1> Vector12d;
typedef DiagonalMatrix<double, 12> DiagonalMatrix12d;
typedef DiagonalMatrix<double, 6> DiagonalMatrix6d;

}  // namespace Eigen

class TargetTrackerKF {
  // The pose state as a SE3 object
  Sophus::SE3d T_;
  // The velocity state
  Sophus::Vector6d v_ = Sophus::Vector6d::Zero();

  // The process covariance matrix
  Eigen::DiagonalMatrix12d Q_;

  // The observation covariance matrix
  Eigen::DiagonalMatrix6d R_;

  // The error covariance matrix
  Eigen::Matrix12d P_ = 0.01 * Eigen::Matrix12d::Identity();

  // Caches / Aliases the zero and identity matrices
  const Sophus::Matrix6d zero_ = Sophus::Matrix6d::Zero();
  const Sophus::Matrix6d iden_ = Sophus::Matrix6d::Identity();

 public:
  // Default constructor. This exists to be compatible with std::map and std::unordered_map's operator[] as this
  // operator default-constructs the object when called with a nonexistent key
  TargetTrackerKF()
      : T_(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()),
        Q_(Eigen::Vector12d::Ones().asDiagonal()),
        R_(Sophus::Vector6d::Ones().asDiagonal()){};

  // Regular constructor
  // Initializes the pose representation Tm and lets the user initialize the process and observation covariance
  // matrices, which are defaulted to identity otherwise
  TargetTrackerKF(const Sophus::SE3d &Tm, const Eigen::DiagonalMatrix12d &Q = Eigen::Vector12d::Ones().asDiagonal(),
                  const Eigen::DiagonalMatrix6d &R = Sophus::Vector6d::Ones().asDiagonal())
      : T_(Tm), Q_(Q), R_(R){};

  // Run the process model. This is simply a discrete time 6DoF rigid-body equation of motion written in SE3 language
  void processModel(double dT) {
    auto xi = Sophus::SE3d::exp(-dT * v_);
    T_ = xi * T_;
    auto F = (Eigen::Matrix12d() << xi.Adj(), zero_, zero_, iden_).finished();
    P_ = F * P_ * F.transpose() + dT * dT * Q_.toDenseMatrix();
  }

  // Runs the observation model. Gets the Pose measurement, calculates the innovation using SE3 log maps, then updates
  // the state estimation
  void observationModel(const Sophus::SE3d &Tm) {
    auto Zv = (T_ * Tm.inverse()).log();

    auto Hm = (Eigen::Matrix<double, 6, 12>() << Sophus::Matrix6d::Identity(), Sophus::Matrix6d::Zero()).finished();

    const Eigen::Matrix<double, 12, 6> Km =
        P_ * Hm.transpose() * (Hm * P_ * Hm.transpose() + R_.toDenseMatrix()).inverse();
    const Eigen::Vector12d Psi = Km * Zv;
    T_ = Sophus::SE3d::exp(-Psi.head<6>()) * T_;
    v_ += Psi.tail<6>();
    P_ = (Eigen::Matrix12d::Identity() - Km * Hm) * P_;
  }

  // Accesses the pose state
  const Sophus::SE3d &pose() const { return T_; }

  // Accesses the velocity state
  const Sophus::Vector6d &velocity() const { return v_; }

  // Retrieves the covariance matrix
  const Eigen::Matrix12d &covariance() const { return P_; }
};