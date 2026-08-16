/**
 *  @file   NhcFactor.cpp
 *  @brief  Implementation of the non-holonomic constraint (NHC) factors
 *  @date   July 4, 2026
 **/

#include <gtsam/navigation/NhcFactor.h>

#include <iostream>

namespace gtsam {

namespace {
/// Lever-arm-corrected body-frame velocity and its pose/velocity Jacobians:
///   v_body = R_bw * v_world + gyro x leverArm     (R_bw = pose.rotation()^T)
/// Only the rotation block of the pose Jacobian is non-zero; a right
/// perturbation of the pose rotation gives d(v_body) = [v_imu]_x.
Vector3 bodyVelocity(const Pose3& pose, const Vector3& v_w, const Vector3& gyro,
                     const Vector3& leverArm, OptionalMatrixType H_pose,
                     OptionalMatrixType H_velocity) {
  const Matrix3 R_bw = pose.rotation().matrix().transpose();  // world -> body
  const Vector3 v_imu = R_bw * v_w;
  if (H_pose) {
    Matrix H = Matrix::Zero(3, 6);
    H.leftCols<3>() = skewSymmetric(v_imu);  // rotation block; translation = 0
    *H_pose = H;
  }
  if (H_velocity) *H_velocity = R_bw;
  return v_imu + gyro.cross(leverArm);
}
}  // namespace

// ---------------------------------------------------------------------------
// NhcFactor (2-key, fixed mounting angle)
// ---------------------------------------------------------------------------

NhcFactor::NhcFactor(Key poseKey, Key velocityKey, const Vector3& gyro,
                     const Vector3& leverArm, double wheelSpeed,
                     const SharedNoiseModel& model, const Vector3& mountAngle)
    : Base(model, poseKey, velocityKey),
      gyro_(gyro),
      leverArm_(leverArm),
      vWheel_(wheelSpeed),
      mountAngle_(mountAngle) {}

void NhcFactor::print(const std::string& s,
                      const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ") << "NhcFactor("
            << keyFormatter(key<1>()) << "," << keyFormatter(key<2>()) << ")\n";
  std::cout << "  gyro:       " << gyro_.transpose() << "\n";
  std::cout << "  leverArm:   " << leverArm_.transpose() << "\n";
  std::cout << "  wheelSpeed: " << vWheel_ << "\n";
  std::cout << "  mountAngle: " << mountAngle_.transpose() << "\n";
  if (noiseModel_) noiseModel_->print("  noise model: ");
}

bool NhcFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Vector3>::Equals(gyro_, e->gyro_, tol) &&
         traits<Vector3>::Equals(leverArm_, e->leverArm_, tol) &&
         std::abs(vWheel_ - e->vWheel_) < tol &&
         traits<Vector3>::Equals(mountAngle_, e->mountAngle_, tol);
}

Vector NhcFactor::evaluateError(const Pose3& pose, const Vector3& velocity,
                                OptionalMatrixType H_pose,
                                OptionalMatrixType H_velocity) const {
  const Vector3 v_b =
      bodyVelocity(pose, velocity, gyro_, leverArm_, H_pose, H_velocity);
  // Forward-only odometry rotated into the body frame by the fixed mounting
  // rotation (exact, so any known/calibrated angle is handled correctly).
  const Vector3 predicted =
      Rot3::Expmap(mountAngle_).rotate(Vector3(vWheel_, 0.0, 0.0));
  return v_b - predicted;
}

// ---------------------------------------------------------------------------
// NhcFactorCalib (3-key, estimated mounting angle)
// ---------------------------------------------------------------------------

NhcFactorCalib::NhcFactorCalib(Key poseKey, Key velocityKey, Key mountAngleKey,
                               const Vector3& gyro, const Vector3& leverArm,
                               double wheelSpeed, const SharedNoiseModel& model)
    : Base(model, poseKey, velocityKey, mountAngleKey),
      gyro_(gyro),
      leverArm_(leverArm),
      vWheel_(wheelSpeed) {}

void NhcFactorCalib::print(const std::string& s,
                           const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ") << "NhcFactorCalib("
            << keyFormatter(key<1>()) << "," << keyFormatter(key<2>()) << ","
            << keyFormatter(key<3>()) << ")\n";
  std::cout << "  gyro:       " << gyro_.transpose() << "\n";
  std::cout << "  leverArm:   " << leverArm_.transpose() << "\n";
  std::cout << "  wheelSpeed: " << vWheel_ << "\n";
  if (noiseModel_) noiseModel_->print("  noise model: ");
}

bool NhcFactorCalib::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Vector3>::Equals(gyro_, e->gyro_, tol) &&
         traits<Vector3>::Equals(leverArm_, e->leverArm_, tol) &&
         std::abs(vWheel_ - e->vWheel_) < tol;
}

Vector NhcFactorCalib::evaluateError(const Pose3& pose, const Vector3& velocity,
                                     const Vector3& mountAngle,
                                     OptionalMatrixType H_pose,
                                     OptionalMatrixType H_velocity,
                                     OptionalMatrixType H_mountAngle) const {
  const Vector3 v_b =
      bodyVelocity(pose, velocity, gyro_, leverArm_, H_pose, H_velocity);

  // predicted = Expmap(mountAngle) * [wheel, 0, 0]  (exact rotation)
  const Vector3 v_odo(vWheel_, 0.0, 0.0);
  Vector3 predicted;
  if (H_mountAngle) {
    Matrix3 Hexp, Hrot;
    const Rot3 C = Rot3::Expmap(mountAngle, Hexp);
    predicted = C.rotate(v_odo, Hrot);
    *H_mountAngle = -(Hrot * Hexp);  // d(v_b - predicted)/dphi
  } else {
    predicted = Rot3::Expmap(mountAngle).rotate(v_odo);
  }

  return v_b - predicted;
}

}  // namespace gtsam
