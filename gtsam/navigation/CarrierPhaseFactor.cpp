/**
 *  @file   CarrierPhaseFactor.cpp
 *  @brief  Implementation file for GNSS Carrier Phase factors
 *  @date   March 23, 2026
 **/

#include "CarrierPhaseFactor.h"

#include <limits>

namespace {

/// Speed of light in a vacuum (m/s):
constexpr double CLIGHT = 299792458.0;

}  // namespace

namespace gtsam {

//***************************************************************************
CarrierPhaseFactor::CarrierPhaseFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key ambiguityKey, const double measuredCarrierPhase,
    const Point3& satellitePosition, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhase, satellitePosition,
                       satelliteClockBias} {}

//***************************************************************************
void CarrierPhaseFactor::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(carrierPhase_, "carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool CarrierPhaseFactor::equals(const NonlinearFactor& expected,
                                double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(carrierPhase_, e->carrierPhase_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector CarrierPhaseFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    const double& ambiguity, OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType Hambiguity) const {
  // error = range + c*(dt_u - dt_s) + ambiguity - measurement
  const Vector3 position_difference = receiverPosition - satPos_;
  const double range = position_difference.norm();
  const double rho =
      range + CLIGHT * (receiverClockBias - satClkBias_) + ambiguity;
  const double error = rho - carrierPhase_;

  if (HreceiverPos) {
    if (range < std::numeric_limits<double>::epsilon()) {
      *HreceiverPos = Matrix13::Zero();
    } else {
      *HreceiverPos = (position_difference / range).transpose();
    }
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * CLIGHT;
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
CarrierPhaseFactorArm::CarrierPhaseFactorArm(
    const Key poseKey, const Key receiverClockBiasKey, const Key ambiguityKey,
    const double measuredCarrierPhase, const Point3& satellitePosition,
    const Point3& leverArm, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhase, satellitePosition,
                       satelliteClockBias},
      bL_(leverArm) {}

//***************************************************************************
CarrierPhaseFactorArm::CarrierPhaseFactorArm(
    const Key poseKey, const Key receiverClockBiasKey, const Key ambiguityKey,
    const double measuredCarrierPhase, const Point3& satellitePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, ambiguityKey),
      CarrierPhaseBase{measuredCarrierPhase, satellitePosition,
                       satelliteClockBias},
      bL_(leverArm),
      ecef_T_nav_(ecef_T_nav) {}

//***************************************************************************
void CarrierPhaseFactorArm::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(carrierPhase_, "carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(bL_), "lever arm (body frame meters): ");
  if (ecef_T_nav_) {
    ecef_T_nav_->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool CarrierPhaseFactorArm::equals(const NonlinearFactor& expected,
                                   double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(carrierPhase_, e->carrierPhase_, tol))
    return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<double>::Equals(satClkBias_, e->satClkBias_, tol)) return false;
  if (!traits<Point3>::Equals(bL_, e->bL_, tol)) return false;
  if (ecef_T_nav_.has_value() != e->ecef_T_nav_.has_value()) return false;
  if (ecef_T_nav_ && !ecef_T_nav_->equals(*e->ecef_T_nav_, tol)) return false;
  return true;
}

//***************************************************************************
Vector CarrierPhaseFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    const double& ambiguity, OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType Hambiguity) const {
  // Convert from local nav frame to ECEF if ecef_T_nav is provided:
  Matrix66 H_compose;
  const bool has_nav = ecef_T_nav_.has_value();
  const Pose3 ecef_T_body = has_nav
      ? ecef_T_nav_->compose(pose, {}, H_pose ? &H_compose : nullptr)
      : pose;

  // Compute antenna position in the ECEF frame:
  const Matrix3 ecef_R_body = ecef_T_body.rotation().matrix();
  const Point3 antennaPos = ecef_T_body.translation() + ecef_R_body * bL_;

  // error = range + c*(dt_u - dt_s) + ambiguity - measurement
  const Vector3 position_difference = antennaPos - satPos_;
  const double range = position_difference.norm();
  const double rho =
      range + CLIGHT * (receiverClockBias - satClkBias_) + ambiguity;
  const double error = rho - carrierPhase_;

  // Compute associated derivatives:
  if (H_pose) {
    H_pose->resize(1, 6);
    if (range < std::numeric_limits<double>::epsilon()) {
      H_pose->setZero();
    } else {
      const Matrix u = (position_difference / range).transpose();  // 1x3
      Matrix16 H_ecef;
      H_ecef.block<1, 3>(0, 0) =
          u * (-ecef_R_body * skewSymmetric(bL_));
      H_ecef.block<1, 3>(0, 3) = u * ecef_R_body;
      *H_pose = has_nav ? H_ecef * H_compose : H_ecef;
    }
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * CLIGHT;
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
CarrierPhaseDDFactorArm::CarrierPhaseDDFactorArm(
    const Key poseKey, const Key ambiguityKey, const Key ambiguityBaseKey,
    const double ddCarrierPhase, const Point3& satellitePosition,
    const Point3& baseSatellitePosition, const Point3& referencePosition,
    const Point3& leverArm, const SharedNoiseModel& model)
    : Base(model, poseKey, ambiguityKey, ambiguityBaseKey),
      ddPhase_(ddCarrierPhase),
      satPos_(satellitePosition),
      satPosBase_(baseSatellitePosition),
      refPos_(referencePosition),
      bL_(leverArm) {}

//***************************************************************************
CarrierPhaseDDFactorArm::CarrierPhaseDDFactorArm(
    const Key poseKey, const Key ambiguityKey, const Key ambiguityBaseKey,
    const double ddCarrierPhase, const Point3& satellitePosition,
    const Point3& baseSatellitePosition, const Point3& referencePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const SharedNoiseModel& model)
    : Base(model, poseKey, ambiguityKey, ambiguityBaseKey),
      ddPhase_(ddCarrierPhase),
      satPos_(satellitePosition),
      satPosBase_(baseSatellitePosition),
      refPos_(referencePosition),
      bL_(leverArm),
      ecef_T_nav_(ecef_T_nav) {}

//***************************************************************************
void CarrierPhaseDDFactorArm::print(const std::string& s,
                                    const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(ddPhase_, "DD carrier phase (m): ");
  gtsam::print(Vector(satPos_), "target sat position (ECEF meters): ");
  gtsam::print(Vector(satPosBase_), "base sat position (ECEF meters): ");
  gtsam::print(Vector(refPos_), "reference position (ECEF meters): ");
  gtsam::print(Vector(bL_), "lever arm (body frame meters): ");
  if (ecef_T_nav_) {
    ecef_T_nav_->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool CarrierPhaseDDFactorArm::equals(const NonlinearFactor& expected,
                                     double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(ddPhase_, e->ddPhase_, tol)) return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<Point3>::Equals(satPosBase_, e->satPosBase_, tol)) return false;
  if (!traits<Point3>::Equals(refPos_, e->refPos_, tol)) return false;
  if (!traits<Point3>::Equals(bL_, e->bL_, tol)) return false;
  if (ecef_T_nav_.has_value() != e->ecef_T_nav_.has_value()) return false;
  if (ecef_T_nav_ && !ecef_T_nav_->equals(*e->ecef_T_nav_, tol)) return false;
  return true;
}

//***************************************************************************
Vector CarrierPhaseDDFactorArm::evaluateError(
    const Pose3& pose, const double& ambiguity, const double& ambiguityBase,
    OptionalMatrixType H_pose, OptionalMatrixType Hambiguity,
    OptionalMatrixType HambiguityBase) const {
  // Convert from local nav frame to ECEF if ecef_T_nav is provided:
  Matrix66 H_compose;
  const bool has_nav = ecef_T_nav_.has_value();
  const Pose3 ecef_T_body = has_nav
      ? ecef_T_nav_->compose(pose, {}, H_pose ? &H_compose : nullptr)
      : pose;

  // Compute rover antenna position in ECEF:
  const Matrix3 ecef_R_body = ecef_T_body.rotation().matrix();
  const Point3 antennaPos = ecef_T_body.translation() + ecef_R_body * bL_;

  // DD ranges: rover sees target sat and base sat
  const Vector3 diff_rov = antennaPos - satPos_;
  const double rho_rov = diff_rov.norm();
  const Vector3 diff_rov_base = antennaPos - satPosBase_;
  const double rho_rov_base = diff_rov_base.norm();

  // Reference station ranges (fixed, no derivatives):
  const double rho_ref = (refPos_ - satPos_).norm();
  const double rho_ref_base = (refPos_ - satPosBase_).norm();

  // DD model: rho_rov - rho_ref - rho_rov_base + rho_ref_base + amb - amb_base
  const double dd_rho = rho_rov - rho_ref - rho_rov_base + rho_ref_base;
  const double error = dd_rho + ambiguity - ambiguityBase - ddPhase_;

  // Compute derivatives w.r.t. pose:
  if (H_pose) {
    H_pose->resize(1, 6);
    const bool range_ok =
        rho_rov > std::numeric_limits<double>::epsilon() &&
        rho_rov_base > std::numeric_limits<double>::epsilon();
    if (!range_ok) {
      H_pose->setZero();
    } else {
      // Unit vectors from rover antenna to each satellite
      const Matrix13 u = (diff_rov / rho_rov).transpose();
      const Matrix13 u_base = (diff_rov_base / rho_rov_base).transpose();
      // DD Jacobian w.r.t. ECEF antenna position: u - u_base
      const Matrix13 dd_u = u - u_base;
      // Jacobian w.r.t. pose (rotation + translation with lever arm)
      Matrix16 H_ecef;
      H_ecef.block<1, 3>(0, 0) =
          dd_u * (-ecef_R_body * skewSymmetric(bL_));
      H_ecef.block<1, 3>(0, 3) = dd_u * ecef_R_body;
      *H_pose = has_nav ? H_ecef * H_compose : H_ecef;
    }
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  if (HambiguityBase) {
    *HambiguityBase = -I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
CarrierPhaseDDIonoFactor::CarrierPhaseDDIonoFactor(
    const Key receiverPositionKey, const Key ambiguityKey,
    const Key ambiguityBaseKey, const Key ionoKey,
    const double ddCarrierPhase, const Point3& satellitePosition,
    const Point3& baseSatellitePosition, const Point3& referencePosition,
    const double ionosphereCoefficient, const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, ambiguityKey, ambiguityBaseKey, ionoKey),
      ddPhase_(ddCarrierPhase),
      satPos_(satellitePosition),
      satPosBase_(baseSatellitePosition),
      refPos_(referencePosition),
      ionoCoeff_(ionosphereCoefficient) {}

//***************************************************************************
void CarrierPhaseDDIonoFactor::print(const std::string& s,
                                     const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(ddPhase_, "DD carrier phase (m): ");
  gtsam::print(Vector(satPos_), "target sat position (ECEF meters): ");
  gtsam::print(Vector(satPosBase_), "base sat position (ECEF meters): ");
  gtsam::print(Vector(refPos_), "reference position (ECEF meters): ");
  gtsam::print(ionoCoeff_, "iono coefficient: ");
}

//***************************************************************************
bool CarrierPhaseDDIonoFactor::equals(const NonlinearFactor& expected,
                                      double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(ddPhase_, e->ddPhase_, tol)) return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<Point3>::Equals(satPosBase_, e->satPosBase_, tol)) return false;
  if (!traits<Point3>::Equals(refPos_, e->refPos_, tol)) return false;
  if (!traits<double>::Equals(ionoCoeff_, e->ionoCoeff_, tol)) return false;
  return true;
}

//***************************************************************************
Vector CarrierPhaseDDIonoFactor::evaluateError(
    const Point3& receiverPosition, const double& ambiguity,
    const double& ambiguityBase, const double& iono,
    OptionalMatrixType HreceiverPos, OptionalMatrixType Hambiguity,
    OptionalMatrixType HambiguityBase, OptionalMatrixType Hiono) const {
  const Vector3 diff_rov = receiverPosition - satPos_;
  const double rho_rov = diff_rov.norm();
  const Vector3 diff_rov_base = receiverPosition - satPosBase_;
  const double rho_rov_base = diff_rov_base.norm();

  const double rho_ref = (refPos_ - satPos_).norm();
  const double rho_ref_base = (refPos_ - satPosBase_).norm();

  const double dd_rho = rho_rov - rho_ref - rho_rov_base + rho_ref_base;
  const double error =
      dd_rho + ambiguity - ambiguityBase - ionoCoeff_ * iono - ddPhase_;

  if (HreceiverPos) {
    const bool range_ok =
        rho_rov > std::numeric_limits<double>::epsilon() &&
        rho_rov_base > std::numeric_limits<double>::epsilon();
    if (!range_ok) {
      *HreceiverPos = Matrix13::Zero();
    } else {
      const Matrix13 u = (diff_rov / rho_rov).transpose();
      const Matrix13 u_base = (diff_rov_base / rho_rov_base).transpose();
      *HreceiverPos = u - u_base;
    }
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  if (HambiguityBase) {
    *HambiguityBase = -I_1x1;
  }

  if (Hiono) {
    *Hiono = -I_1x1 * ionoCoeff_;
  }

  return Vector1(error);
}

//***************************************************************************
CarrierPhaseSDFactor::CarrierPhaseSDFactor(
    const Key positionKey, const Key clockKey, const Key ambiguityKey,
    const double sdCarrierPhase, const Point3& satellitePosition,
    const Point3& basePosition, const SharedNoiseModel& model)
    : Base(model, positionKey, clockKey, ambiguityKey),
      sdPhase_(sdCarrierPhase),
      satPos_(satellitePosition),
      basePos_(basePosition) {}

//***************************************************************************
void CarrierPhaseSDFactor::print(const std::string& s,
                                 const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(sdPhase_, "SD carrier phase (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(Vector(basePos_), "base position (ECEF meters): ");
}

//***************************************************************************
bool CarrierPhaseSDFactor::equals(const NonlinearFactor& expected,
                                  double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(sdPhase_, e->sdPhase_, tol)) return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<Point3>::Equals(basePos_, e->basePos_, tol)) return false;
  return true;
}

//***************************************************************************
Vector CarrierPhaseSDFactor::evaluateError(
    const Point3& position, const double& clockBias,
    const double& ambiguity, OptionalMatrixType Hposition,
    OptionalMatrixType HclockBias, OptionalMatrixType Hambiguity) const {
  // SD range: |rover - sat| - |base - sat|
  const Vector3 diff_rov = position - satPos_;
  const double rho_rov = diff_rov.norm();
  const double rho_base = (basePos_ - satPos_).norm();
  const double sd_range = rho_rov - rho_base;

  // error = sd_range + c*dt + N - sd_phi
  constexpr double CLIGHT = 299792458.0;
  const double error = sd_range + CLIGHT * clockBias + ambiguity - sdPhase_;

  if (Hposition) {
    if (rho_rov < std::numeric_limits<double>::epsilon()) {
      *Hposition = Matrix13::Zero();
    } else {
      *Hposition = (diff_rov / rho_rov).transpose();
    }
  }

  if (HclockBias) {
    *HclockBias = I_1x1 * CLIGHT;
  }

  if (Hambiguity) {
    *Hambiguity = I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
CarrierPhaseDDFactor::CarrierPhaseDDFactor(
    const Key positionKey, const Key ambTargetKey, const Key ambRefKey,
    const double sdPhiTarget, const double sdPhiRef,
    const Point3& satPosTarget, const Point3& satPosRef,
    const Point3& basePosition, const SharedNoiseModel& model)
    : Base(model, positionKey, ambTargetKey, ambRefKey),
      sdPhiTarget_(sdPhiTarget), sdPhiRef_(sdPhiRef),
      satPosTarget_(satPosTarget), satPosRef_(satPosRef),
      basePos_(basePosition) {}

//***************************************************************************
void CarrierPhaseDDFactor::print(const std::string& s,
                                        const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(sdPhiTarget_, "SD phi target (m): ");
  gtsam::print(sdPhiRef_, "SD phi ref (m): ");
}

//***************************************************************************
bool CarrierPhaseDDFactor::equals(const NonlinearFactor& expected,
                                         double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(sdPhiTarget_, e->sdPhiTarget_, tol)) return false;
  if (!traits<double>::Equals(sdPhiRef_, e->sdPhiRef_, tol)) return false;
  if (!traits<Point3>::Equals(satPosTarget_, e->satPosTarget_, tol)) return false;
  if (!traits<Point3>::Equals(satPosRef_, e->satPosRef_, tol)) return false;
  if (!traits<Point3>::Equals(basePos_, e->basePos_, tol)) return false;
  return true;
}

//***************************************************************************
Vector CarrierPhaseDDFactor::evaluateError(
    const Point3& position, const double& ambTarget, const double& ambRef,
    OptionalMatrixType Hposition, OptionalMatrixType HambTarget,
    OptionalMatrixType HambRef) const {
  // SD residuals computed from undifferenced observations:
  // sd_resid = (|pos - sat| - |base - sat|) + N - sd_phi
  const Vector3 d_target = position - satPosTarget_;
  const double r_target = d_target.norm();
  const double r_base_target = (basePos_ - satPosTarget_).norm();
  const double sd_resid_target = (r_target - r_base_target) + ambTarget - sdPhiTarget_;

  const Vector3 d_ref = position - satPosRef_;
  const double r_ref = d_ref.norm();
  const double r_base_ref = (basePos_ - satPosRef_).norm();
  const double sd_resid_ref = (r_ref - r_base_ref) + ambRef - sdPhiRef_;

  // DD = SD_target - SD_ref
  const double error = sd_resid_target - sd_resid_ref;

  if (Hposition) {
    const bool ok = r_target > std::numeric_limits<double>::epsilon() &&
                    r_ref > std::numeric_limits<double>::epsilon();
    if (!ok) {
      *Hposition = Matrix13::Zero();
    } else {
      const Matrix13 u_target = (d_target / r_target).transpose();
      const Matrix13 u_ref = (d_ref / r_ref).transpose();
      *Hposition = u_target - u_ref;
    }
  }

  if (HambTarget) {
    *HambTarget = I_1x1;
  }

  if (HambRef) {
    *HambRef = -I_1x1;
  }

  return Vector1(error);
}

}  // namespace gtsam
