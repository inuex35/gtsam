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
 * Clock biases are in seconds (same convention as PseudorangeFactorArm).
 * Ambiguity is in meters (= lambda * N). To recover the integer
 * ambiguity N, divide by the wavelength: N = ambiguity_meters / lambda
 */
struct CarrierPhaseBase {
  double carrierPhase_;  ///< Carrier phase measurement in meters.
  Point3 satPos_;        ///< Satellite position in WGS84 ECEF meters.
  double satClkBias_;    ///< Satellite clock bias in seconds.
};

/**
 * Undifferenced GNSS carrier phase factor for point positioning.
 *
 * The error model is:
 *   error = ||recv_pos - satPos|| + c*(dt_u - dt_s) + ambiguity - phi
 *
 * where dt_u, dt_s are in seconds and ambiguity is in meters.
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
   * @param receiverClockBiasKey Receiver clock bias node (seconds).
   * @param ambiguityKey Ambiguity node (meters, = lambda * N)..
   * @param measuredCarrierPhase Carrier phase measurement in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param satelliteClockBias Satellite clock bias in seconds.
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
 * The error model is:
 *   error = ||antenna_pos - satPos|| + c*(dt_u - dt_s) + ambiguity - phi
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
   */
  CarrierPhaseFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhase, const Point3& satellitePosition,
      const Point3& leverArm, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /**
   * Construct a CarrierPhaseFactorArm with ecef_T_nav (local nav frame pose).
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

/**
 * Double-differenced carrier phase factor with lever arm correction.
 *
 * Implements the RTK carrier phase model using double differences between
 * rover/reference stations and target/base satellites. Eliminates receiver
 * clock biases and reduces atmospheric errors.
 *
 * The DD observation (precomputed by caller):
 *   dd_phi = phi_rov - phi_ref - phi_rov_base + phi_ref_base
 *
 * The DD model:
 *   dd_rho = ||antenna_rov - sat|| - ||ref_pos - sat||
 *          - ||antenna_rov - sat_base|| + ||ref_pos - sat_base||
 *
 * The error:
 *   error = dd_rho + ambiguity - ambiguity_base - dd_phi
 *
 * Keys: (Pose3 rover_pose, double ambiguity, double ambiguity_base)
 * Fixed: satellite positions (2), reference station position, DD measurement.
 *
 * When the optional ecef_T_nav transform is provided, the pose key is
 * interpreted as a local navigation frame pose.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseDDFactorArm
    : public NoiseModelFactorN<Pose3, double, double> {
 private:
  typedef NoiseModelFactorN<Pose3, double, double> Base;

  double ddPhase_;     ///< DD carrier phase measurement in meters.
  Point3 satPos_;      ///< Target satellite ECEF position in meters.
  Point3 satPosBase_;  ///< Base satellite ECEF position in meters.
  Point3 refPos_;      ///< Reference station ECEF position in meters.
  Point3 bL_;          ///< Lever arm from body origin to antenna in body frame.
  std::optional<Pose3> ecef_T_nav_;  ///< Optional ECEF-from-nav transform.

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseDDFactorArm> shared_ptr;
  typedef CarrierPhaseDDFactorArm This;

  /** default constructor - only use for serialization */
  CarrierPhaseDDFactorArm()
      : ddPhase_(0.0), satPos_(0, 0, 0), satPosBase_(0, 0, 0),
        refPos_(0, 0, 0), bL_(0, 0, 0) {}

  virtual ~CarrierPhaseDDFactorArm() = default;

  /**
   * Construct a CarrierPhaseDDFactorArm (ECEF pose key).
   *
   * @param poseKey Rover Pose3 key (body pose in ECEF frame).
   * @param ambiguityKey DD ambiguity for target satellite (meters).
   * @param ambiguityBaseKey DD ambiguity for base satellite (meters).
   * @param ddCarrierPhase DD carrier phase measurement in meters.
   * @param satellitePosition Target satellite ECEF position in meters.
   * @param baseSatellitePosition Base satellite ECEF position in meters.
   * @param referencePosition Reference station ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param model 1-D noise model.
   */
  CarrierPhaseDDFactorArm(
      Key poseKey, Key ambiguityKey, Key ambiguityBaseKey,
      double ddCarrierPhase,
      const Point3& satellitePosition, const Point3& baseSatellitePosition,
      const Point3& referencePosition, const Point3& leverArm,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /**
   * Construct a CarrierPhaseDDFactorArm with ecef_T_nav.
   */
  CarrierPhaseDDFactorArm(
      Key poseKey, Key ambiguityKey, Key ambiguityBaseKey,
      double ddCarrierPhase,
      const Point3& satellitePosition, const Point3& baseSatellitePosition,
      const Point3& referencePosition, const Point3& leverArm,
      const Pose3& ecef_T_nav,
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
                       const double& ambiguity,
                       const double& ambiguityBase,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType Hambiguity,
                       OptionalMatrixType HambiguityBase) const override;

  /// return the lever arm
  inline const Point3& leverArm() const { return bL_; }

  /// return the optional ecef_T_nav transform
  inline const std::optional<Pose3>& ecefTnav() const { return ecef_T_nav_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseDDFactorArm::Base);
    ar& BOOST_SERIALIZATION_NVP(ddPhase_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satPosBase_);
    ar& BOOST_SERIALIZATION_NVP(refPos_);
    ar& BOOST_SERIALIZATION_NVP(bL_);
    ar& BOOST_SERIALIZATION_NVP(ecef_T_nav_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseDDFactorArm>
    : public Testable<CarrierPhaseDDFactorArm> {};

/**
 * Double-differenced carrier phase factor with ionosphere estimation.
 *
 * Adds a DD ionosphere delay parameter to enable L1/L2 decorrelation.
 * The ionosphere delay has opposite signs on L1 vs L2, allowing the
 * solver to separate ambiguity from ionosphere using dual-frequency data.
 *
 * The error model:
 *   error = dd_rho + ambiguity - ambiguity_base - ionoCoeff * iono - dd_phi
 *
 * where ionoCoeff = 1.0 for L1, (f1/f2)^2 for L2.
 * iono is the DD ionosphere delay in meters at L1 frequency.
 *
 * Keys: (Point3 rover_position, double ambiguity, double ambiguity_base,
 *        double iono)
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseDDIonoFactor
    : public NoiseModelFactorN<Point3, double, double, double> {
 private:
  typedef NoiseModelFactorN<Point3, double, double, double> Base;

  double ddPhase_;     ///< DD carrier phase measurement in meters.
  Point3 satPos_;      ///< Target satellite ECEF position in meters.
  Point3 satPosBase_;  ///< Base satellite ECEF position in meters.
  Point3 refPos_;      ///< Reference station ECEF position in meters.
  double ionoCoeff_;   ///< Ionosphere coefficient: 1.0 for L1, (f1/f2)^2 for L2.

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseDDIonoFactor> shared_ptr;
  typedef CarrierPhaseDDIonoFactor This;

  CarrierPhaseDDIonoFactor()
      : ddPhase_(0.0), satPos_(0, 0, 0), satPosBase_(0, 0, 0),
        refPos_(0, 0, 0), ionoCoeff_(1.0) {}

  virtual ~CarrierPhaseDDIonoFactor() = default;

  /**
   * @param receiverPositionKey Rover Point3 ECEF position node.
   * @param ambiguityKey DD ambiguity for target satellite (meters).
   * @param ambiguityBaseKey DD ambiguity for base satellite (meters).
   * @param ionoKey DD ionosphere delay at L1 frequency (meters).
   * @param ddCarrierPhase DD carrier phase measurement in meters.
   * @param satellitePosition Target satellite ECEF position in meters.
   * @param baseSatellitePosition Base satellite ECEF position in meters.
   * @param referencePosition Reference station ECEF position in meters.
   * @param ionosphereCoefficient 1.0 for L1, (f1/f2)^2 for L2.
   * @param model 1-D noise model.
   */
  CarrierPhaseDDIonoFactor(
      Key receiverPositionKey, Key ambiguityKey, Key ambiguityBaseKey,
      Key ionoKey, double ddCarrierPhase,
      const Point3& satellitePosition, const Point3& baseSatellitePosition,
      const Point3& referencePosition, double ionosphereCoefficient = 1.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Point3& receiverPosition,
                       const double& ambiguity,
                       const double& ambiguityBase,
                       const double& iono,
                       OptionalMatrixType HreceiverPos,
                       OptionalMatrixType Hambiguity,
                       OptionalMatrixType HambiguityBase,
                       OptionalMatrixType Hiono) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseDDIonoFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(ddPhase_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satPosBase_);
    ar& BOOST_SERIALIZATION_NVP(refPos_);
    ar& BOOST_SERIALIZATION_NVP(ionoCoeff_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseDDIonoFactor>
    : public Testable<CarrierPhaseDDIonoFactor> {};

/**
 * Single-differenced carrier phase factor (RTKLIB style).
 *
 * SD observation = L_rover - L_base for each satellite.
 * Estimates SD ambiguity per satellite. DD formed at LAMBDA time.
 *
 * The error model:
 *   error = (|pos - sat| - |base - sat|) + c * dt + ambiguity - sdPhase
 *
 * Keys: (Point3 rover_position, double clock_bias, double sd_ambiguity)
 * Fixed: satellite position, base station position, SD carrier phase.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseSDFactor
    : public NoiseModelFactorN<Point3, double, double> {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

  double sdPhase_;   ///< SD carrier phase measurement in meters.
  Point3 satPos_;    ///< Satellite ECEF position in meters.
  Point3 basePos_;   ///< Base station ECEF position in meters.

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseSDFactor> shared_ptr;
  typedef CarrierPhaseSDFactor This;

  CarrierPhaseSDFactor()
      : sdPhase_(0.0), satPos_(0, 0, 0), basePos_(0, 0, 0) {}

  virtual ~CarrierPhaseSDFactor() = default;

  /**
   * @param positionKey Rover Point3 ECEF position node.
   * @param clockKey Rover-base clock bias (seconds).
   * @param ambiguityKey SD ambiguity (meters).
   * @param sdCarrierPhase SD carrier phase measurement (meters).
   * @param satellitePosition Satellite ECEF position (meters).
   * @param basePosition Base station ECEF position (meters).
   * @param model 1-D noise model.
   */
  CarrierPhaseSDFactor(
      Key positionKey, Key clockKey, Key ambiguityKey,
      double sdCarrierPhase, const Point3& satellitePosition,
      const Point3& basePosition,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Point3& position,
                       const double& clockBias,
                       const double& ambiguity,
                       OptionalMatrixType Hposition,
                       OptionalMatrixType HclockBias,
                       OptionalMatrixType Hambiguity) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseSDFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(sdPhase_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(basePos_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseSDFactor>
    : public Testable<CarrierPhaseSDFactor> {};

/**
 * GREAT-FGO style DD carrier phase factor.
 *
 * Computes DD residual from undifferenced observations internally.
 * Takes raw SD carrier phase for target and reference satellite.
 * SD ambiguity parameters are correctly aligned with observations.
 *
 * Internal computation:
 *   sd_resid_target = |pos - sat_target| - |base - sat_target| + N_target - sd_phi_target
 *   sd_resid_ref    = |pos - sat_ref|    - |base - sat_ref|    + N_ref    - sd_phi_ref
 *   dd_error = sd_resid_target - sd_resid_ref
 *
 * Keys: (Point3 position, double N_target, double N_ref)
 * Fixed: satellite positions, base position, SD carrier phases.
 *
 * N_target and N_ref are SD ambiguities initialized as:
 *   N = sd_phi - (|approx - sat| - |base - sat|)
 * which gives N ≈ true SD ambiguity.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseDDFactor
    : public NoiseModelFactorN<Point3, double, double> {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

  double sdPhiTarget_;    ///< SD carrier phase for target satellite (meters).
  double sdPhiRef_;       ///< SD carrier phase for reference satellite (meters).
  Point3 satPosTarget_;   ///< Target satellite ECEF position.
  Point3 satPosRef_;      ///< Reference satellite ECEF position.
  Point3 basePos_;        ///< Base station ECEF position.

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseDDFactor> shared_ptr;
  typedef CarrierPhaseDDFactor This;

  CarrierPhaseDDFactor()
      : sdPhiTarget_(0), sdPhiRef_(0),
        satPosTarget_(0,0,0), satPosRef_(0,0,0), basePos_(0,0,0) {}

  virtual ~CarrierPhaseDDFactor() = default;

  /**
   * @param positionKey Rover Point3 ECEF position.
   * @param ambTargetKey SD ambiguity for target satellite (meters).
   * @param ambRefKey SD ambiguity for reference satellite (meters).
   * @param sdPhiTarget SD carrier phase for target (meters): L_rov - L_base.
   * @param sdPhiRef SD carrier phase for ref (meters): L_rov - L_base.
   * @param satPosTarget Target satellite ECEF position.
   * @param satPosRef Reference satellite ECEF position.
   * @param basePosition Base station ECEF position.
   * @param model 1-D noise model.
   */
  CarrierPhaseDDFactor(
      Key positionKey, Key ambTargetKey, Key ambRefKey,
      double sdPhiTarget, double sdPhiRef,
      const Point3& satPosTarget, const Point3& satPosRef,
      const Point3& basePosition,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Point3& position,
                       const double& ambTarget,
                       const double& ambRef,
                       OptionalMatrixType Hposition,
                       OptionalMatrixType HambTarget,
                       OptionalMatrixType HambRef) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseDDFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(sdPhiTarget_);
    ar& BOOST_SERIALIZATION_NVP(sdPhiRef_);
    ar& BOOST_SERIALIZATION_NVP(satPosTarget_);
    ar& BOOST_SERIALIZATION_NVP(satPosRef_);
    ar& BOOST_SERIALIZATION_NVP(basePos_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseDDFactor>
    : public Testable<CarrierPhaseDDFactor> {};

}  // namespace gtsam
