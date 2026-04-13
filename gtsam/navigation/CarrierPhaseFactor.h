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
 * DD carrier phase factor (GICI style).
 *
 * Takes SD (rover-base) carrier phase observations for ref and target
 * satellites, satellite positions, base station position, and wavelength.
 * Computes DD and geometric distances with Sagnac correction internally.
 *
 * error = (sdCpRef - sdCpTarget)
 *       - [(geodist(satRef,pos) - geodist(satRef,base))
 *        - (geodist(satTarget,pos) - geodist(satTarget,base))]
 *       - lam * (ambRef - ambTarget)
 *
 * @param positionKey   Rover Point3 ECEF position.
 * @param ambRefKey     SD ambiguity of ref satellite [m].
 * @param ambTargetKey  SD ambiguity of target satellite [m].
 * @param sdCpRef       SD carrier phase for ref satellite [m]: CP_rov - CP_base.
 * @param sdCpTarget    SD carrier phase for target satellite [m].
 * @param satRef        Reference satellite ECEF position [m].
 * @param satTarget     Target satellite ECEF position [m].
 * @param basePos       Base station ECEF position [m].
 * @param lam           Wavelength [m/cycle].
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DDCarrierPhaseFactor
    : public NoiseModelFactorN<Point3, double, double> {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

  double sdCpRef_;       ///< SD carrier phase for ref satellite [m]
  double sdCpTarget_;    ///< SD carrier phase for target satellite [m]
  Point3 satRef_;        ///< Reference satellite ECEF position [m]
  Point3 satTarget_;     ///< Target satellite ECEF position [m]
  Point3 basePos_;       ///< Base station ECEF position [m]
  double lam_;           ///< Wavelength [m/cycle]

  static constexpr double OMGE = 7.2921151467e-5;
  static constexpr double C_LIGHT = 299792458.0;

  static double geodist(const Point3& sat, const Point3& rcv, Point3& e) {
    const Point3 dr = sat - rcv;
    const double r = dr.norm();
    e = dr / r;
    return r + OMGE * (sat.x() * rcv.y() - sat.y() * rcv.x()) / C_LIGHT;
  }

 public:
  using Base::evaluateError;
  typedef std::shared_ptr<DDCarrierPhaseFactor> shared_ptr;
  typedef DDCarrierPhaseFactor This;

  DDCarrierPhaseFactor()
      : sdCpRef_(0), sdCpTarget_(0),
        satRef_(0, 0, 0), satTarget_(0, 0, 0), basePos_(0, 0, 0), lam_(0) {}

  virtual ~DDCarrierPhaseFactor() = default;

  DDCarrierPhaseFactor(Key positionKey, Key ambRefKey, Key ambTargetKey,
                       double sdCpRef, double sdCpTarget,
                       const Point3& satRef, const Point3& satTarget,
                       const Point3& basePos, double lam,
                       const SharedNoiseModel& model = noiseModel::Unit::Create(1))
      : Base(model, positionKey, ambRefKey, ambTargetKey),
        sdCpRef_(sdCpRef), sdCpTarget_(sdCpTarget),
        satRef_(satRef), satTarget_(satTarget), basePos_(basePos), lam_(lam) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ") << "DDCarrierPhaseFactor\n";
    std::cout << "  sdCpRef: " << sdCpRef_ << " sdCpTarget: " << sdCpTarget_
              << " lam: " << lam_ << "\n";
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           std::abs(sdCpRef_ - e->sdCpRef_) < tol &&
           std::abs(sdCpTarget_ - e->sdCpTarget_) < tol &&
           std::abs(lam_ - e->lam_) < tol &&
           traits<Point3>::Equals(satRef_, e->satRef_, tol) &&
           traits<Point3>::Equals(satTarget_, e->satTarget_, tol) &&
           traits<Point3>::Equals(basePos_, e->basePos_, tol);
  }

  Vector evaluateError(const Point3& pos,
                       const double& ambRef, const double& ambTarget,
                       OptionalMatrixType Hpos,
                       OptionalMatrixType HambRef,
                       OptionalMatrixType HambTarget) const override {
    // DD observation
    const double ddObs = sdCpRef_ - sdCpTarget_;

    // Rover geometric distances
    Point3 eRef, eTarget;
    const double rRovRef = geodist(satRef_, pos, eRef);
    const double rRovTarget = geodist(satTarget_, pos, eTarget);

    // Base geometric distances (fixed)
    Point3 dummy;
    const double rBaseRef = geodist(satRef_, basePos_, dummy);
    const double rBaseTarget = geodist(satTarget_, basePos_, dummy);

    // DD model
    const double ddModel = (rRovRef - rBaseRef) - (rRovTarget - rBaseTarget);

    const double error = ddObs - ddModel - lam_ * (ambRef - ambTarget);

    if (Hpos) {
      *Hpos = (Matrix(1, 3) << (eRef - eTarget).transpose()).finished();
    }
    if (HambRef) {
      *HambRef = (Matrix(1, 1) << -lam_).finished();
    }
    if (HambTarget) {
      *HambTarget = (Matrix(1, 1) << lam_).finished();
    }

    return Vector1(error);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(DDCarrierPhaseFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(sdCpRef_);
    ar& BOOST_SERIALIZATION_NVP(sdCpTarget_);
    ar& BOOST_SERIALIZATION_NVP(satRef_);
    ar& BOOST_SERIALIZATION_NVP(satTarget_);
    ar& BOOST_SERIALIZATION_NVP(basePos_);
    ar& BOOST_SERIALIZATION_NVP(lam_);
  }
#endif
};

template <>
struct traits<DDCarrierPhaseFactor> : public Testable<DDCarrierPhaseFactor> {};

}  // namespace gtsam
