/**
 *  @file   NhcFactor.h
 *  @brief  Non-holonomic constraint (NHC) factors for wheeled-vehicle odometry
 *  @date   July 4, 2026
 **/
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {

/**
 * Non-holonomic constraint (NHC) factor for a wheeled vehicle.
 *
 * Enforces that, after correcting for the IMU->body lever arm and a (fixed)
 * IMU mounting misalignment, the body-frame velocity matches a forward-only
 * wheel odometry velocity (i.e. no lateral / vertical slip):
 *
 *   v_body    = R_wb^T * v_world + gyro x leverArm   (lever-arm corrected)
 *   v_odo     = [wheelSpeed, 0, 0]                    (forward-only odometry)
 *   predicted = Rot3::Expmap(phi) * v_odo             (phi = mounting angle)
 *   error     = v_body - predicted                    (3-vector)
 *
 * Here the mounting misalignment @c phi is a *fixed constructor constant* (it
 * is not estimated). Pass Vector3::Zero() (the default) for a plain NHC, or a
 * known/calibrated value to apply a fixed mounting correction. The rotation is
 * applied exactly (Rot3::Expmap), so any known angle is handled correctly. Use
 * NhcFactorCalib instead to co-estimate the mounting angle online.
 *
 * Connects two variables:
 *   - Pose3   body-in-world pose (only the rotation enters the error);
 *   - Vector3 world-frame velocity.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT NhcFactor : public NoiseModelFactorN<Pose3, Vector3> {
 private:
  typedef NoiseModelFactorN<Pose3, Vector3> Base;

  Vector3 gyro_;        ///< body-frame angular rate [rad/s]
  Vector3 leverArm_;    ///< IMU->body lever arm [m], body frame
  double vWheel_;       ///< forward wheel speed [m/s]
  Vector3 mountAngle_;  ///< fixed IMU mounting misalignment [rad]

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<NhcFactor> shared_ptr;
  typedef NhcFactor This;

  /** default constructor - only use for serialization */
  NhcFactor()
      : gyro_(Vector3::Zero()),
        leverArm_(Vector3::Zero()),
        vWheel_(0.0),
        mountAngle_(Vector3::Zero()) {}

  ~NhcFactor() override = default;

  /**
   * Construct an NhcFactor (fixed / non-estimated mounting angle).
   *
   * @param poseKey     body-in-world Pose3 node.
   * @param velocityKey world-frame Vector3 velocity node.
   * @param gyro        body-frame angular rate [rad/s].
   * @param leverArm    IMU->body lever arm [m], body frame.
   * @param wheelSpeed  forward wheel odometry speed [m/s].
   * @param model       3-D noise model (forward, lateral, vertical).
   * @param mountAngle  fixed IMU mounting misalignment [rad] (default zero).
   */
  NhcFactor(Key poseKey, Key velocityKey, const Vector3& gyro,
            const Vector3& leverArm, double wheelSpeed,
            const SharedNoiseModel& model,
            const Vector3& mountAngle = Vector3::Zero());

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& pose, const Vector3& velocity,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType H_velocity) const override;

  inline const Vector3& gyro() const { return gyro_; }
  inline const Vector3& leverArm() const { return leverArm_; }
  inline double wheelSpeed() const { return vWheel_; }
  inline const Vector3& mountAngle() const { return mountAngle_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(gyro_);
    ar& BOOST_SERIALIZATION_NVP(leverArm_);
    ar& BOOST_SERIALIZATION_NVP(vWheel_);
    ar& BOOST_SERIALIZATION_NVP(mountAngle_);
  }
#endif
};

/// traits
template <>
struct traits<NhcFactor> : public Testable<NhcFactor> {};

/**
 * Non-holonomic constraint factor that co-estimates the IMU mounting angle.
 *
 * Same error model as NhcFactor (predicted = Rot3::Expmap(phi) * v_odo), but
 * the mounting misalignment @c phi is an estimated Vector3 state (a persistent
 * calibration variable, typically shared across epochs and anchored by a loose
 * prior). Use this for online mounting calibration (e.g. an arbitrarily-mounted
 * device).
 *
 * Note on observability: a forward-only NHC does not observe the component of
 * @c phi about the forward axis (rotating [v,0,0] about x is a no-op), so that
 * degree of freedom must be constrained by a prior or another sensor.
 *
 * Connects three variables:
 *   - Pose3   body-in-world pose (only the rotation enters the error);
 *   - Vector3 world-frame velocity;
 *   - Vector3 IMU mounting misalignment angle [rad] (small-angle model).
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT NhcFactorCalib
    : public NoiseModelFactorN<Pose3, Vector3, Vector3> {
 private:
  typedef NoiseModelFactorN<Pose3, Vector3, Vector3> Base;

  Vector3 gyro_;      ///< body-frame angular rate [rad/s]
  Vector3 leverArm_;  ///< IMU->body lever arm [m], body frame
  double vWheel_;     ///< forward wheel speed [m/s]

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<NhcFactorCalib> shared_ptr;
  typedef NhcFactorCalib This;

  /** default constructor - only use for serialization */
  NhcFactorCalib()
      : gyro_(Vector3::Zero()), leverArm_(Vector3::Zero()), vWheel_(0.0) {}

  ~NhcFactorCalib() override = default;

  /**
   * Construct an NhcFactorCalib (estimated mounting angle).
   *
   * @param poseKey       body-in-world Pose3 node.
   * @param velocityKey   world-frame Vector3 velocity node.
   * @param mountAngleKey IMU mounting misalignment Vector3 [rad] node.
   * @param gyro          body-frame angular rate [rad/s].
   * @param leverArm      IMU->body lever arm [m], body frame.
   * @param wheelSpeed    forward wheel odometry speed [m/s].
   * @param model         3-D noise model (forward, lateral, vertical).
   */
  NhcFactorCalib(Key poseKey, Key velocityKey, Key mountAngleKey,
                 const Vector3& gyro, const Vector3& leverArm,
                 double wheelSpeed, const SharedNoiseModel& model);

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& pose, const Vector3& velocity,
                       const Vector3& mountAngle, OptionalMatrixType H_pose,
                       OptionalMatrixType H_velocity,
                       OptionalMatrixType H_mountAngle) const override;

  inline const Vector3& gyro() const { return gyro_; }
  inline const Vector3& leverArm() const { return leverArm_; }
  inline double wheelSpeed() const { return vWheel_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(gyro_);
    ar& BOOST_SERIALIZATION_NVP(leverArm_);
    ar& BOOST_SERIALIZATION_NVP(vWheel_);
  }
#endif
};

/// traits
template <>
struct traits<NhcFactorCalib> : public Testable<NhcFactorCalib> {};

}  // namespace gtsam
