/**
 *  @file   CarrierPhaseFactor.h
 *  @brief  Header file for GNSS Carrier Phase factors
 *  @date   March 23, 2026
 **/
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>
#include <string>

namespace gtsam {

/**
 * Base class storing common members for carrier phase factors.
 *
 * All quantities are in meters. The receiver clock bias and ambiguity
 * variables are also in meters, following the convention used by gici-open.
 * This makes the factor frequency-independent and simplifies multi-frequency
 * processing. To recover the integer ambiguity N, divide by the wavelength:
 *   N = ambiguity_meters / lambda
 */
struct CarrierPhaseBase {
  double carrierPhase_;  ///< Carrier phase measurement in meters.
  Point3 satPos_;        ///< Satellite position in WGS84 ECEF meters.
  double satClkBias_;    ///< Satellite clock bias in meters (c * dt_s).
};

/**
 * Undifferenced GNSS carrier phase factor for point positioning.
 *
 * The error model (all in meters) is:
 *   error = ||recv_pos - satPos|| + clock_bias - sat_clock_bias
 *           + ambiguity - carrier_phase
 *
 * where clock_bias and ambiguity are estimated in meters.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseFactor
    : public NoiseModelFactorN<Point3, double, double>,
      private CarrierPhaseBase {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseFactor> shared_ptr;
  typedef CarrierPhaseFactor This;

  /** default constructor - only use for serialization */
  CarrierPhaseFactor()
      : CarrierPhaseBase{0.0, Point3(0, 0, 0), 0.0} {}

  virtual ~CarrierPhaseFactor() = default;

  /**
   * Construct a CarrierPhaseFactor.
   *
   * @param receiverPositionKey Receiver gtsam::Point3 ECEF position node.
   * @param receiverClockBiasKey Receiver clock bias node (meters).
   * @param ambiguityKey Ambiguity node (meters, = lambda * N).
   * @param measuredCarrierPhase Carrier phase measurement in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param satelliteClockBias Satellite clock bias in meters (c * dt_s).
   * @param model 1-D noise model.
   */
  CarrierPhaseFactor(
      Key receiverPositionKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhase, const Point3& satellitePosition,
      double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

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
  Vector evaluateError(const Point3& receiverPosition,
                       const double& receiverClockBias,
                       const double& ambiguity,
                       OptionalMatrixType HreceiverPos,
                       OptionalMatrixType HreceiverClockBias,
                       OptionalMatrixType Hambiguity) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(carrierPhase_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseFactor> : public Testable<CarrierPhaseFactor> {};

/**
 * Carrier phase factor with lever arm correction.
 *
 * Like CarrierPhaseFactor, but uses a Pose3 (position + attitude) as the
 * receiver state variable, allowing compensation for a lever arm offset.
 *
 * The antenna position is computed as:
 *   antenna_pos = ecef_T_body.translation() + ecef_R_body * bL_
 *
 * The error model (all in meters) is:
 *   error = ||antenna_pos - satPos|| + clock_bias - sat_clock_bias
 *           + ambiguity - carrier_phase
 *
 * When the optional ecef_T_nav transform is provided, the pose key is
 * interpreted as a local navigation frame pose (e.g., ENU), and the factor
 * internally converts it to ECEF via ecef_T_body = ecef_T_nav * nav_T_body.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseFactorArm
    : public NoiseModelFactorN<Pose3, double, double>,
      private CarrierPhaseBase {
 private:
  typedef NoiseModelFactorN<Pose3, double, double> Base;

  Point3 bL_;  ///< Lever arm from body origin to antenna in body frame.
  std::optional<Pose3> ecef_T_nav_;  ///< Optional ECEF-from-nav transform.

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseFactorArm> shared_ptr;
  typedef CarrierPhaseFactorArm This;

  /** default constructor - only use for serialization */
  CarrierPhaseFactorArm()
      : CarrierPhaseBase{0.0, Point3(0, 0, 0), 0.0}, bL_(0, 0, 0) {}

  virtual ~CarrierPhaseFactorArm() = default;

  /**
   * Construct a CarrierPhaseFactorArm (ECEF pose key).
   *
   * @param poseKey Receiver gtsam::Pose3 key (body pose in ECEF frame).
   * @param receiverClockBiasKey Receiver clock bias node (meters).
   * @param ambiguityKey Ambiguity node (meters, = lambda * N).
   * @param measuredCarrierPhase Carrier phase measurement in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param satelliteClockBias Satellite clock bias in meters (c * dt_s).
   * @param model 1-D noise model.
   */
  CarrierPhaseFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhase, const Point3& satellitePosition,
      const Point3& leverArm, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /**
   * Construct a CarrierPhaseFactorArm with ecef_T_nav (local nav frame pose).
   *
   * @param poseKey Receiver gtsam::Pose3 key (body pose in local nav frame).
   * @param receiverClockBiasKey Receiver clock bias node (meters).
   * @param ambiguityKey Ambiguity node (meters, = lambda * N).
   * @param measuredCarrierPhase Carrier phase measurement in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param ecef_T_nav Transform from local navigation frame to ECEF.
   * @param satelliteClockBias Satellite clock bias in meters (c * dt_s).
   * @param model 1-D noise model.
   */
  CarrierPhaseFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhase, const Point3& satellitePosition,
      const Point3& leverArm, const Pose3& ecef_T_nav,
      double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

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
  Vector evaluateError(const Pose3& pose,
                       const double& receiverClockBias,
                       const double& ambiguity,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType HreceiverClockBias,
                       OptionalMatrixType Hambiguity) const override;

  /// return the lever arm
  inline const Point3& leverArm() const { return bL_; }

  /// return the optional ecef_T_nav transform
  inline const std::optional<Pose3>& ecefTnav() const { return ecef_T_nav_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseFactorArm::Base);
    ar& BOOST_SERIALIZATION_NVP(carrierPhase_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
    ar& BOOST_SERIALIZATION_NVP(bL_);
    ar& BOOST_SERIALIZATION_NVP(ecef_T_nav_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseFactorArm>
    : public Testable<CarrierPhaseFactorArm> {};

}  // namespace gtsam
