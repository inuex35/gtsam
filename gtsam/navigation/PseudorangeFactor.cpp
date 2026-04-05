/**
 *  @file   PseudorangeFactor.cpp
 *  @author Sammy Guo
 *  @brief  Implementation file for GNSS Pseudorange factor
 *  @date   January 18, 2026
 **/

#include "PseudorangeFactor.h"

#include <limits>

namespace {

/// Speed of light in a vacuum (m/s):
constexpr double CLIGHT = 299792458.0;

}  // namespace

namespace gtsam {

//***************************************************************************
PseudorangeFactor::PseudorangeFactor(const Key receiverPositionKey,
                                     const Key receiverClockBiasKey,
                                     const double measuredPseudorange,
                                     const Point3& satellitePosition,
                                     const double satelliteClockBias,
                                     const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias} {}

//***************************************************************************
void PseudorangeFactor::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool PseudorangeFactor::equals(const NonlinearFactor& expected,
                               double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(pseudorange_, e->pseudorange_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector PseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClockBias,
    OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias) const {
  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = receiverPosition - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - pseudorange_;

  // Compute associated derivatives:
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

  return Vector1(error);
}

//***************************************************************************
DifferentialPseudorangeFactor::DifferentialPseudorangeFactor(
    const Key receiverPositionKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, receiverClockBiasKey,
           differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias} {}

//***************************************************************************
void DifferentialPseudorangeFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
}

//***************************************************************************
bool DifferentialPseudorangeFactor::equals(const NonlinearFactor& expected,
                                           double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<double>::Equals(pseudorange_, e->pseudorange_, tol) &&
         traits<Point3>::Equals(satPos_, e->satPos_, tol) &&
         traits<double>::Equals(satClkBias_, e->satClkBias_, tol);
}

//***************************************************************************
Vector DifferentialPseudorangeFactor::evaluateError(
    const Point3& receiverPosition, const double& receiverClock_bias,
    const double& differentialCorrection, OptionalMatrixType HreceiverPos,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HdifferentialCorrection) const {
  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = receiverPosition - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiverClock_bias - satClkBias_);
  const double error = rho - pseudorange_ - differentialCorrection;

  // Compute associated derivatives:
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

  if (HdifferentialCorrection) {
    *HdifferentialCorrection = -I_1x1;
  }

  return Vector1(error);
}
//***************************************************************************
PseudorangeFactorArm::PseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const Point3& leverArm, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      bL_(leverArm) {}

//***************************************************************************
PseudorangeFactorArm::PseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const double measuredPseudorange, const Point3& satellitePosition,
    const Point3& leverArm, const Pose3& ecef_T_nav,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      bL_(leverArm),
      ecef_T_nav_(ecef_T_nav) {}

//***************************************************************************
void PseudorangeFactorArm::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(bL_), "lever arm (body frame meters): ");
  if (ecef_T_nav_) {
    ecef_T_nav_->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool PseudorangeFactorArm::equals(const NonlinearFactor& expected,
                                   double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(pseudorange_, e->pseudorange_, tol)) return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<double>::Equals(satClkBias_, e->satClkBias_, tol)) return false;
  if (!traits<Point3>::Equals(bL_, e->bL_, tol)) return false;
  if (ecef_T_nav_.has_value() != e->ecef_T_nav_.has_value()) return false;
  if (ecef_T_nav_ && !ecef_T_nav_->equals(*e->ecef_T_nav_, tol)) return false;
  return true;
}

//***************************************************************************
Vector PseudorangeFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias) const {
  // Convert from local nav frame to ECEF if ecef_T_nav is provided:
  Matrix66 H_compose;
  const bool has_nav = ecef_T_nav_.has_value();
  const Pose3 ecef_T_body = has_nav
      ? ecef_T_nav_->compose(pose, {}, H_pose ? &H_compose : nullptr)
      : pose;

  // Compute antenna position in the ECEF frame:
  const Matrix3 ecef_R_body = ecef_T_body.rotation().matrix();
  const Point3 antennaPos = ecef_T_body.translation() + ecef_R_body * bL_;

  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = antennaPos - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - pseudorange_;

  // Compute associated derivatives:
  if (H_pose) {
    H_pose->resize(1, 6);
    if (range < std::numeric_limits<double>::epsilon()) {
      H_pose->setZero();
    } else {
      // u = unit vector from satellite to antenna
      const Matrix u = (position_difference / range).transpose();  // 1x3
      Matrix16 H_ecef;
      H_ecef.block<1, 3>(0, 0) =
          u * (-ecef_R_body * skewSymmetric(bL_));
      H_ecef.block<1, 3>(0, 3) = u * ecef_R_body;
      // Chain rule: if ecef_T_nav is set, multiply by compose Jacobian
      *H_pose = has_nav ? H_ecef * H_compose : H_ecef;
    }
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * CLIGHT;
  }

  return Vector1(error);
}

//***************************************************************************
DifferentialPseudorangeFactorArm::DifferentialPseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const Point3& leverArm,
    const double satelliteClockBias, const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      bL_(leverArm) {}

//***************************************************************************
DifferentialPseudorangeFactorArm::DifferentialPseudorangeFactorArm(
    const Key poseKey, const Key receiverClockBiasKey,
    const Key differentialCorrectionKey, const double measuredPseudorange,
    const Point3& satellitePosition, const Point3& leverArm,
    const Pose3& ecef_T_nav, const double satelliteClockBias,
    const SharedNoiseModel& model)
    : Base(model, poseKey, receiverClockBiasKey, differentialCorrectionKey),
      PseudorangeBase{measuredPseudorange, satellitePosition,
                      satelliteClockBias},
      bL_(leverArm),
      ecef_T_nav_(ecef_T_nav) {}

//***************************************************************************
void DifferentialPseudorangeFactorArm::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(pseudorange_, "pseudorange (m): ");
  gtsam::print(Vector(satPos_), "sat position (ECEF meters): ");
  gtsam::print(satClkBias_, "sat clock bias (s): ");
  gtsam::print(Vector(bL_), "lever arm (body frame meters): ");
  if (ecef_T_nav_) {
    ecef_T_nav_->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool DifferentialPseudorangeFactorArm::equals(
    const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(pseudorange_, e->pseudorange_, tol)) return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<double>::Equals(satClkBias_, e->satClkBias_, tol)) return false;
  if (!traits<Point3>::Equals(bL_, e->bL_, tol)) return false;
  if (ecef_T_nav_.has_value() != e->ecef_T_nav_.has_value()) return false;
  if (ecef_T_nav_ && !ecef_T_nav_->equals(*e->ecef_T_nav_, tol)) return false;
  return true;
}

//***************************************************************************
Vector DifferentialPseudorangeFactorArm::evaluateError(
    const Pose3& pose, const double& receiverClockBias,
    const double& differentialCorrection, OptionalMatrixType H_pose,
    OptionalMatrixType HreceiverClockBias,
    OptionalMatrixType HdifferentialCorrection) const {
  // Convert from local nav frame to ECEF if ecef_T_nav is provided:
  Matrix66 H_compose;
  const bool has_nav = ecef_T_nav_.has_value();
  const Pose3 ecef_T_body = has_nav
      ? ecef_T_nav_->compose(pose, {}, H_pose ? &H_compose : nullptr)
      : pose;

  // Compute antenna position in the ECEF frame:
  const Matrix3 ecef_R_body = ecef_T_body.rotation().matrix();
  const Point3 antennaPos = ecef_T_body.translation() + ecef_R_body * bL_;

  // Apply pseudorange equation: rho = range + c*[dt_u - dt^s]
  const Vector3 position_difference = antennaPos - satPos_;
  const double range = position_difference.norm();
  const double rho = range + CLIGHT * (receiverClockBias - satClkBias_);
  const double error = rho - pseudorange_ - differentialCorrection;

  // Compute associated derivatives:
  if (H_pose) {
    H_pose->resize(1, 6);
    if (range < std::numeric_limits<double>::epsilon()) {
      H_pose->setZero();
    } else {
      // u = unit vector from satellite to antenna
      const Matrix u = (position_difference / range).transpose();  // 1x3
      Matrix16 H_ecef;
      H_ecef.block<1, 3>(0, 0) =
          u * (-ecef_R_body * skewSymmetric(bL_));
      H_ecef.block<1, 3>(0, 3) = u * ecef_R_body;
      // Chain rule: if ecef_T_nav is set, multiply by compose Jacobian
      *H_pose = has_nav ? H_ecef * H_compose : H_ecef;
    }
  }

  if (HreceiverClockBias) {
    *HreceiverClockBias = I_1x1 * CLIGHT;
  }

  if (HdifferentialCorrection) {
    *HdifferentialCorrection = -I_1x1;
  }

  return Vector1(error);
}

//***************************************************************************
PseudorangeDDFactor::PseudorangeDDFactor(
    const Key receiverPositionKey, const Key ionoKey,
    const double sdPrTarget, const double sdPrRef,
    const Point3& satellitePosition, const Point3& baseSatellitePosition,
    const Point3& referencePosition,
    const double ionosphereCoefficient,
    const SharedNoiseModel& model)
    : Base(model, receiverPositionKey, ionoKey),
      sdPrTarget_(sdPrTarget), sdPrRef_(sdPrRef),
      satPos_(satellitePosition),
      satPosBase_(baseSatellitePosition),
      refPos_(referencePosition),
      ionoCoeff_(ionosphereCoefficient) {}

//***************************************************************************
void PseudorangeDDFactor::print(const std::string& s,
                                const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(Vector(satPos_), "target sat position (ECEF meters): ");
  gtsam::print(Vector(satPosBase_), "base sat position (ECEF meters): ");
  gtsam::print(Vector(refPos_), "reference position (ECEF meters): ");
}

//***************************************************************************
bool PseudorangeDDFactor::equals(const NonlinearFactor& expected,
                                 double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(sdPrTarget_, e->sdPrTarget_, tol)) return false;
  if (!traits<double>::Equals(sdPrRef_, e->sdPrRef_, tol)) return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<Point3>::Equals(satPosBase_, e->satPosBase_, tol)) return false;
  if (!traits<Point3>::Equals(refPos_, e->refPos_, tol)) return false;
  return true;
}

//***************************************************************************
Vector PseudorangeDDFactor::evaluateError(
    const Point3& receiverPosition, const double& ddIono,
    OptionalMatrixType HreceiverPos, OptionalMatrixType Hiono) const {
  const Vector3 diff_rov = receiverPosition - satPos_;
  const double rho_rov = diff_rov.norm();
  const Vector3 diff_rov_base = receiverPosition - satPosBase_;
  const double rho_rov_base = diff_rov_base.norm();

  const double rho_ref = (refPos_ - satPos_).norm();
  const double rho_ref_base = (refPos_ - satPosBase_).norm();

  const double dd_rho = rho_rov - rho_ref - rho_rov_base + rho_ref_base;
  const double dd_pr = sdPrTarget_ - sdPrRef_;
  const double error = dd_rho + ionoCoeff_ * ddIono - dd_pr;

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

  if (Hiono) {
    *Hiono = Vector1(ionoCoeff_).transpose();
  }

  return Vector1(error);
}

//***************************************************************************
PseudorangeDDFactorArm::PseudorangeDDFactorArm(
    const Key poseKey, const double ddPseudorange,
    const Point3& satellitePosition, const Point3& baseSatellitePosition,
    const Point3& referencePosition, const Point3& leverArm,
    const SharedNoiseModel& model)
    : Base(model, poseKey),
      ddPseudorange_(ddPseudorange),
      satPos_(satellitePosition),
      satPosBase_(baseSatellitePosition),
      refPos_(referencePosition),
      bL_(leverArm) {}

//***************************************************************************
PseudorangeDDFactorArm::PseudorangeDDFactorArm(
    const Key poseKey, const double ddPseudorange,
    const Point3& satellitePosition, const Point3& baseSatellitePosition,
    const Point3& referencePosition, const Point3& leverArm,
    const Pose3& ecef_T_nav, const SharedNoiseModel& model)
    : Base(model, poseKey),
      ddPseudorange_(ddPseudorange),
      satPos_(satellitePosition),
      satPosBase_(baseSatellitePosition),
      refPos_(referencePosition),
      bL_(leverArm),
      ecef_T_nav_(ecef_T_nav) {}

//***************************************************************************
void PseudorangeDDFactorArm::print(const std::string& s,
                                   const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  gtsam::print(ddPseudorange_, "DD pseudorange (m): ");
  gtsam::print(Vector(satPos_), "target sat position (ECEF meters): ");
  gtsam::print(Vector(satPosBase_), "base sat position (ECEF meters): ");
  gtsam::print(Vector(refPos_), "reference position (ECEF meters): ");
  gtsam::print(Vector(bL_), "lever arm (body frame meters): ");
  if (ecef_T_nav_) {
    ecef_T_nav_->print("ecef_T_nav:\n");
  }
}

//***************************************************************************
bool PseudorangeDDFactorArm::equals(const NonlinearFactor& expected,
                                    double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  if (e == nullptr || !Base::equals(*e, tol)) return false;
  if (!traits<double>::Equals(ddPseudorange_, e->ddPseudorange_, tol))
    return false;
  if (!traits<Point3>::Equals(satPos_, e->satPos_, tol)) return false;
  if (!traits<Point3>::Equals(satPosBase_, e->satPosBase_, tol)) return false;
  if (!traits<Point3>::Equals(refPos_, e->refPos_, tol)) return false;
  if (!traits<Point3>::Equals(bL_, e->bL_, tol)) return false;
  if (ecef_T_nav_.has_value() != e->ecef_T_nav_.has_value()) return false;
  if (ecef_T_nav_ && !ecef_T_nav_->equals(*e->ecef_T_nav_, tol)) return false;
  return true;
}

//***************************************************************************
Vector PseudorangeDDFactorArm::evaluateError(
    const Pose3& pose, OptionalMatrixType H_pose) const {
  // Convert from local nav frame to ECEF if ecef_T_nav is provided:
  Matrix66 H_compose;
  const bool has_nav = ecef_T_nav_.has_value();
  const Pose3 ecef_T_body = has_nav
      ? ecef_T_nav_->compose(pose, {}, H_pose ? &H_compose : nullptr)
      : pose;

  // Compute rover antenna position in ECEF:
  const Matrix3 ecef_R_body = ecef_T_body.rotation().matrix();
  const Point3 antennaPos = ecef_T_body.translation() + ecef_R_body * bL_;

  // DD ranges:
  const Vector3 diff_rov = antennaPos - satPos_;
  const double rho_rov = diff_rov.norm();
  const Vector3 diff_rov_base = antennaPos - satPosBase_;
  const double rho_rov_base = diff_rov_base.norm();

  const double rho_ref = (refPos_ - satPos_).norm();
  const double rho_ref_base = (refPos_ - satPosBase_).norm();

  const double dd_rho = rho_rov - rho_ref - rho_rov_base + rho_ref_base;
  const double error = dd_rho - ddPseudorange_;

  if (H_pose) {
    H_pose->resize(1, 6);
    const bool range_ok =
        rho_rov > std::numeric_limits<double>::epsilon() &&
        rho_rov_base > std::numeric_limits<double>::epsilon();
    if (!range_ok) {
      H_pose->setZero();
    } else {
      const Matrix13 u = (diff_rov / rho_rov).transpose();
      const Matrix13 u_base = (diff_rov_base / rho_rov_base).transpose();
      const Matrix13 dd_u = u - u_base;
      Matrix16 H_ecef;
      H_ecef.block<1, 3>(0, 0) =
          dd_u * (-ecef_R_body * skewSymmetric(bL_));
      H_ecef.block<1, 3>(0, 3) = dd_u * ecef_R_body;
      *H_pose = has_nav ? H_ecef * H_compose : H_ecef;
    }
  }

  return Vector1(error);
}

}  // namespace gtsam
