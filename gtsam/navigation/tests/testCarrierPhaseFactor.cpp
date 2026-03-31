/**
 *  @file   testCarrierPhaseFactor.cpp
 *  @brief  Unit tests for CarrierPhaseFactor and CarrierPhaseFactorArm
 *  @date   March 23, 2026
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cmath>

using namespace gtsam;

/// GPS L1 wavelength (~0.1903 m)
static constexpr double LAMBDA_L1 = 0.190293672798365;
/// Speed of light
static constexpr double CLIGHT = 299792458.0;

/// Test helper: compute ECEF-to-ENU transform from geodetic origin (WGS84).
static Pose3 makeEcefTnav(double lat_deg, double lon_deg, double h) {
  constexpr double deg2rad = M_PI / 180.0;
  const double lat = lat_deg * deg2rad;
  const double lon = lon_deg * deg2rad;
  const double slat = std::sin(lat), clat = std::cos(lat);
  const double slon = std::sin(lon), clon = std::cos(lon);

  Matrix3 R;
  R.col(0) = Vector3(-slon, clon, 0.0);
  R.col(1) = Vector3(-slat * clon, -slat * slon, clat);
  R.col(2) = Vector3(clat * clon, clat * slon, slat);

  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  const double e2 = 2.0 * f - f * f;
  const double N = a / std::sqrt(1.0 - e2 * slat * slat);
  const Point3 t((N + h) * clat * clon,
                 (N + h) * clat * slon,
                 (N * (1.0 - e2) + h) * slat);
  return Pose3(Rot3(R), t);
}

// *************************************************************************
// CarrierPhaseFactor tests (all quantities in meters)
// *************************************************************************
TEST(TestCarrierPhaseFactor, Constructor) {
  // All zeros: error = 0 + 0 - 0 + 0 - 0 = 0
  const auto factor =
      CarrierPhaseFactor(Key(0), Key(1), Key(2), 0.0, Point3::Zero(), 0.0);
  const double error = factor.evaluateError(Point3::Zero(), 0.0, 0.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, AmbiguityEffect) {
  // error = range + clock - sat_clock + ambiguity - measurement
  // With range = measurement and clock = sat_clock = 0:
  //   error = ambiguity
  const Point3 satPos(0.0, 0.0, 20200000.0);
  const Point3 recvPos(0.0, 0.0, 0.0);
  const double range = 20200000.0;

  const auto factor = CarrierPhaseFactor(
      Key(0), Key(1), Key(2), range, satPos, 0.0);

  const double e0 = factor.evaluateError(recvPos, 0.0, 0.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, e0, 1e-9);

  // Adding 1.0 meter ambiguity should change error by 1.0
  const double e1 = factor.evaluateError(recvPos, 0.0, 1.0)[0];
  EXPECT_DOUBLES_EQUAL(1.0, e1, 1e-9);

  // Adding one L1 cycle in meters
  const double amb_one_cycle = LAMBDA_L1;
  const double e2 = factor.evaluateError(recvPos, 0.0, amb_one_cycle)[0];
  EXPECT_DOUBLES_EQUAL(LAMBDA_L1, e2, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, Jacobians) {
  const auto factor = CarrierPhaseFactor(
      Key(0), Key(1), Key(2), 24874028.989,
      Point3(-5824269.46342, -22935011.26952, -12195522.22428),
      -0.00022743876852667193);

  Values values;
  values.insert(Key(0), Point3(-3961908.12, 3348995.59, 3698211.13));
  values.insert(Key(1), 5.377885e-07);         // clock in seconds
  values.insert(Key(2), LAMBDA_L1 * 1000.0);   // ambiguity in meters
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, print) {
  const auto factor =
      CarrierPhaseFactor(Key(0), Key(1), Key(2), 0.0, Point3::Zero(), 0.0);
  factor.print("test ");
}

// *************************************************************************
TEST(TestCarrierPhaseFactor, equals) {
  const auto f1 = CarrierPhaseFactor(1, 2, 3, 100.0, Point3(1, 2, 3), 0.0);
  const auto f2 = CarrierPhaseFactor(1, 2, 3, 100.0, Point3(1, 2, 3), 0.0);
  const auto f3 = CarrierPhaseFactor(1, 2, 3, 200.0, Point3(1, 2, 3), 0.0);

  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
}

// *************************************************************************
// CarrierPhaseFactorArm tests
// *************************************************************************
TEST(TestCarrierPhaseFactorArm, Constructor) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), leverArm, 0.0);

  const Pose3 pose(Rot3::Identity(), Point3::Zero());
  const double error = factor.evaluateError(pose, 0.0, 0.0)[0];
  // Error should be range from lever arm to origin = ||leverArm||
  EXPECT_DOUBLES_EQUAL(leverArm.norm(), error, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, Jacobians) {
  const Point3 leverArm(0.1, 0.0, -0.5);
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 24874028.989,
      Point3(-5824269.46342, -22935011.26952, -12195522.22428),
      leverArm, -0.00022743876852667193);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(-3961908.12, 3348995.59, 3698211.13)));
  values.insert(Key(1), 5.377885e-07);          // clock in seconds
  values.insert(Key(2), LAMBDA_L1 * 1000.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, ZeroLeverArm) {
  const Point3 pos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);
  const double phi = 24874028.989;
  const double satClk = -0.00022743876852667193;
  const double clock = 1e-7;
  const double amb_m = LAMBDA_L1 * 500.0;

  const auto factorPt = CarrierPhaseFactor(
      Key(0), Key(1), Key(2), phi, satPos, satClk);
  const auto factorArm = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, Point3::Zero(), satClk);

  const double errorPt = factorPt.evaluateError(pos, clock, amb_m)[0];
  const double errorArm = factorArm.evaluateError(
      Pose3(Rot3::Identity(), pos), clock, amb_m)[0];
  EXPECT_DOUBLES_EQUAL(errorPt, errorArm, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavIdentity) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const Point3 satPos(0.0, 0.0, 3.0);
  const double phi = 4.0;

  const auto factorEcef = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, leverArm, 0.0);
  const auto factorNav = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, leverArm, Pose3::Identity(), 0.0);

  const Pose3 pose(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0));
  const double errorEcef = factorEcef.evaluateError(pose, 1.0, 10.0)[0];
  const double errorNav = factorNav.evaluateError(pose, 1.0, 10.0)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavIdentityJacobians) {
  const Point3 leverArm(0.5, -0.3, 1.0);
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 4.0, Vector3(0.0, 0.0, 3.0), leverArm,
      Pose3::Identity(), 0.0);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(1.0, 2.0, 3.0)));
  values.insert(Key(1), 0.0);
  values.insert(Key(2), 10.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavENUJacobians) {
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Point3 leverArm(0.1, 0.0, -0.5);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);

  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 24874028.989, satPos, leverArm,
      ecef_T_nav, -0.00022743876852667193);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                               Point3(10.0, 20.0, 5.0)));
  values.insert(Key(1), 5.377885093511699e-07);
  values.insert(Key(2), LAMBDA_L1 * 1000.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavConsistency) {
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Pose3 nav_T_body(Rot3::RzRyRx(0.05, -0.03, 0.1),
                          Point3(10.0, 20.0, 5.0));
  const Pose3 ecef_T_body = ecef_T_nav.compose(nav_T_body);

  const Point3 leverArm(0.1, 0.0, -0.5);
  const Point3 satPos(-5824269.46342, -22935011.26952, -12195522.22428);
  const double phi = 24874028.989;
  const double satClkBias = -0.00022743876852667193;
  const double clockBias = 5.377885093511699e-07;
  const double ambiguity_m = LAMBDA_L1 * 1000.0;

  const auto factorEcef = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, leverArm, satClkBias);
  const auto factorNav = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), phi, satPos, leverArm,
      ecef_T_nav, satClkBias);

  const double errorEcef =
      factorEcef.evaluateError(ecef_T_body, clockBias, ambiguity_m)[0];
  const double errorNav =
      factorNav.evaluateError(nav_T_body, clockBias, ambiguity_m)[0];
  EXPECT_DOUBLES_EQUAL(errorEcef, errorNav, 1e-6);
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, EcefTnavEquals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);

  const auto f1 = CarrierPhaseFactorArm(
      1, 2, 3, 0.0, Point3::Zero(), leverArm, 0.0);
  const auto f2 = CarrierPhaseFactorArm(
      1, 2, 3, 0.0, Point3::Zero(), leverArm, ecef_T_nav, 0.0);

  CHECK(!f1.equals(f2));
  CHECK(f2.equals(f2));
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, print) {
  const auto factor = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), Point3(0.1, 0.2, 0.3),
      0.0);
  factor.print("test ");

  const auto factorNav = CarrierPhaseFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3::Zero(), Point3(0.1, 0.2, 0.3),
      Pose3::Identity(), 0.0);
  factorNav.print("test nav ");
}

// *************************************************************************
TEST(TestCarrierPhaseFactorArm, equals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto f1 = CarrierPhaseFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), leverArm, 0.0);
  const auto f2 = CarrierPhaseFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), leverArm, 0.0);
  const auto f3 = CarrierPhaseFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), leverArm, 999.0);

  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));  // different sat clock bias
}

// *************************************************************************
// CarrierPhaseDDFactor tests (Point3, no lever arm)
// *************************************************************************
TEST(TestCarrierPhaseDDFactor, ZeroError) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);

  const auto factor = CarrierPhaseDDFactor(
      Key(0), Key(1), Key(2), 0.0, satPos, satPosBase, refPos);
  const double error = factor.evaluateError(refPos, 5.0, 5.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-6);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactor, Jacobians) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);

  const auto factor = CarrierPhaseDDFactor(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos);
  Values values;
  values.insert(Key(0), Point3(-3961780.0, 3349050.0, 3698350.0));
  values.insert(Key(1), 50.0);
  values.insert(Key(2), 48.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactor, ConsistencyWithArm) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 pos(-3961780.0, 3349050.0, 3698350.0);

  const auto factorPt = CarrierPhaseDDFactor(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos);
  const auto factorArm = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, Point3::Zero());

  const double errorPt = factorPt.evaluateError(pos, 50.0, 48.0)[0];
  const double errorArm = factorArm.evaluateError(
      Pose3(Rot3::Identity(), pos), 50.0, 48.0)[0];
  EXPECT_DOUBLES_EQUAL(errorPt, errorArm, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactor, print) {
  const auto factor = CarrierPhaseDDFactor(
      Key(0), Key(1), Key(2), 0.0, Point3(1,2,3), Point3(4,5,6), Point3(7,8,9));
  factor.print("test DD CP ");
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactor, equals) {
  const auto f1 = CarrierPhaseDDFactor(1,2,3, 100.0, Point3(1,2,3), Point3(4,5,6), Point3(7,8,9));
  const auto f2 = CarrierPhaseDDFactor(1,2,3, 100.0, Point3(1,2,3), Point3(4,5,6), Point3(7,8,9));
  const auto f3 = CarrierPhaseDDFactor(1,2,3, 200.0, Point3(1,2,3), Point3(4,5,6), Point3(7,8,9));
  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
}

// *************************************************************************
// CarrierPhaseDDFactorArm tests
// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, ZeroError) {
  // When rover is at reference position, DD range = 0, amb = amb_base, error = 0
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 leverArm(0.0, 0.0, 0.0);

  const double rho_rov = (refPos - satPos).norm();
  const double rho_ref = rho_rov;
  const double rho_rov_base = (refPos - satPosBase).norm();
  const double rho_ref_base = rho_rov_base;
  const double dd_phi = rho_rov - rho_ref - rho_rov_base + rho_ref_base;  // = 0

  const auto factor = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), dd_phi, satPos, satPosBase, refPos, leverArm);

  const Pose3 pose(Rot3::Identity(), refPos);
  const double error = factor.evaluateError(pose, 5.0, 5.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-6);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, Jacobians) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 leverArm(0.31, 0.0, -0.55);

  const auto factor = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, leverArm);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3),
                               Point3(-3961780.0, 3349050.0, 3698350.0)));
  values.insert(Key(1), 50.0);
  values.insert(Key(2), 48.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, AmbiguityJacobians) {
  // amb Jacobian = +1, amb_base Jacobian = -1
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 leverArm(0.1, 0.0, 0.0);

  const auto factor = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 0.0, satPos, satPosBase, refPos, leverArm);

  const Pose3 pose(Rot3::Identity(), refPos);
  const double e1 = factor.evaluateError(pose, 10.0, 5.0)[0];
  const double e2 = factor.evaluateError(pose, 11.0, 5.0)[0];
  const double e3 = factor.evaluateError(pose, 10.0, 6.0)[0];
  EXPECT_DOUBLES_EQUAL(1.0, e2 - e1, 1e-9);   // d(error)/d(amb) = +1
  EXPECT_DOUBLES_EQUAL(-1.0, e3 - e1, 1e-9);  // d(error)/d(amb_base) = -1
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, EcefTnavIdentity) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 leverArm(0.31, 0.0, -0.55);

  const auto factorEcef = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, leverArm);
  const auto factorNav = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, leverArm,
      Pose3::Identity());

  const Pose3 pose(Rot3::RzRyRx(0.1, 0.2, 0.3),
                    Point3(-3961780.0, 3349050.0, 3698350.0));
  const double errEcef = factorEcef.evaluateError(pose, 50.0, 48.0)[0];
  const double errNav = factorNav.evaluateError(pose, 50.0, 48.0)[0];
  EXPECT_DOUBLES_EQUAL(errEcef, errNav, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, EcefTnavENUJacobians) {
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 leverArm(0.31, 0.0, -0.55);

  const auto factor = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, leverArm,
      ecef_T_nav);

  Values values;
  values.insert(Key(0), Pose3(Rot3::RzRyRx(0.05, -0.03, 0.1),
                               Point3(10.0, 20.0, 5.0)));
  values.insert(Key(1), 50.0);
  values.insert(Key(2), 48.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, EcefTnavConsistency) {
  const Pose3 ecef_T_nav = makeEcefTnav(35.578, 139.749, 80.0);
  const Pose3 nav_T_body(Rot3::RzRyRx(0.05, -0.03, 0.1),
                          Point3(10.0, 20.0, 5.0));
  const Pose3 ecef_T_body = ecef_T_nav.compose(nav_T_body);

  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);
  const Point3 leverArm(0.31, 0.0, -0.55);

  const auto factorEcef = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, leverArm);
  const auto factorNav = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 100.0, satPos, satPosBase, refPos, leverArm,
      ecef_T_nav);

  const double errEcef = factorEcef.evaluateError(ecef_T_body, 50.0, 48.0)[0];
  const double errNav = factorNav.evaluateError(nav_T_body, 50.0, 48.0)[0];
  EXPECT_DOUBLES_EQUAL(errEcef, errNav, 1e-6);
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, print) {
  const auto factor = CarrierPhaseDDFactorArm(
      Key(0), Key(1), Key(2), 0.0, Point3(1, 2, 3), Point3(4, 5, 6),
      Point3(7, 8, 9), Point3(0.1, 0.2, 0.3));
  factor.print("test DD ");
}

// *************************************************************************
TEST(TestCarrierPhaseDDFactorArm, equals) {
  const Point3 leverArm(0.1, 0.2, 0.3);
  const auto f1 = CarrierPhaseDDFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), Point3(4, 5, 6),
      Point3(7, 8, 9), leverArm);
  const auto f2 = CarrierPhaseDDFactorArm(
      1, 2, 3, 100.0, Point3(1, 2, 3), Point3(4, 5, 6),
      Point3(7, 8, 9), leverArm);
  const auto f3 = CarrierPhaseDDFactorArm(
      1, 2, 3, 200.0, Point3(1, 2, 3), Point3(4, 5, 6),
      Point3(7, 8, 9), leverArm);

  CHECK(f1.equals(f2));
  CHECK(!f1.equals(f3));
}

// *************************************************************************
// CarrierPhaseDDIonoFactor tests
// *************************************************************************
TEST(TestCarrierPhaseDDIonoFactor, ZeroError) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);

  const auto factor = CarrierPhaseDDIonoFactor(
      Key(0), Key(1), Key(2), Key(3), 0.0, satPos, satPosBase, refPos, 1.0);
  const double error = factor.evaluateError(refPos, 5.0, 5.0, 0.0)[0];
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-6);
}

// *************************************************************************
TEST(TestCarrierPhaseDDIonoFactor, IonoEffect) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);

  // L1: ionoCoeff = 1.0
  const auto factorL1 = CarrierPhaseDDIonoFactor(
      Key(0), Key(1), Key(2), Key(3), 0.0, satPos, satPosBase, refPos, 1.0);
  // L2: ionoCoeff = (f1/f2)^2
  const double gamma = (1575.42 / 1227.60) * (1575.42 / 1227.60);
  const auto factorL2 = CarrierPhaseDDIonoFactor(
      Key(0), Key(1), Key(2), Key(3), 0.0, satPos, satPosBase, refPos, gamma);

  const double iono = 1.5;  // 1.5m DD iono delay
  const double e1 = factorL1.evaluateError(refPos, 5.0, 5.0, iono)[0];
  const double e2 = factorL2.evaluateError(refPos, 5.0, 5.0, iono)[0];
  // L1 iono contribution: -1.0 * 1.5 = -1.5
  // L2 iono contribution: -gamma * 1.5
  EXPECT_DOUBLES_EQUAL(-1.5, e1, 1e-9);
  EXPECT_DOUBLES_EQUAL(-gamma * 1.5, e2, 1e-9);
}

// *************************************************************************
TEST(TestCarrierPhaseDDIonoFactor, JacobiansL1) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);

  const auto factor = CarrierPhaseDDIonoFactor(
      Key(0), Key(1), Key(2), Key(3), 100.0, satPos, satPosBase, refPos, 1.0);
  Values values;
  values.insert(Key(0), Point3(-3961780.0, 3349050.0, 3698350.0));
  values.insert(Key(1), 50.0);
  values.insert(Key(2), 48.0);
  values.insert(Key(3), 0.5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseDDIonoFactor, JacobiansL2) {
  const Point3 refPos(-3961908.12, 3348995.59, 3698211.13);
  const Point3 satPos(-5824269.46, -22935011.27, -12195522.22);
  const Point3 satPosBase(15524471.21, -16649826.68, -13512405.95);

  const double gamma = (1575.42 / 1227.60) * (1575.42 / 1227.60);
  const auto factor = CarrierPhaseDDIonoFactor(
      Key(0), Key(1), Key(2), Key(3), 100.0, satPos, satPosBase, refPos, gamma);
  Values values;
  values.insert(Key(0), Point3(-3961780.0, 3349050.0, 3698350.0));
  values.insert(Key(1), 50.0);
  values.insert(Key(2), 48.0);
  values.insert(Key(3), 0.3);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-3, 1e-5);
}

// *************************************************************************
TEST(TestCarrierPhaseDDIonoFactor, L1L2Decorrelation) {
  // Verify L1 and L2 ambiguities decorrelate when iono is estimated
  const Point3 truePos(-3961780.0, 3349050.0, 3698350.0);
  const Point3 refPos(-3961905.0, 3348994.0, 3698212.0);
  const Point3 sat(-5824269.0, -22935011.0, -12195522.0);
  const Point3 satBase(15524471.0, -16649827.0, -13512406.0);

  const double gamma = (1575.42 / 1227.60) * (1575.42 / 1227.60);
  const double lam1 = 299792458.0 / 1575.42e6;
  const double lam2 = 299792458.0 / 1227.60e6;
  const double N_L1 = 5.0 * lam1;
  const double N_L2 = 8.0 * lam2;
  const double trueIono = 0.1;  // 0.1m DD iono

  // Compute DD measurements
  auto dd_rho = [&](const Point3& p) {
    return (p - sat).norm() - (refPos - sat).norm()
         - (p - satBase).norm() + (refPos - satBase).norm();
  };
  const double rho = dd_rho(truePos);
  const double phi_L1 = rho + N_L1 - trueIono;
  const double phi_L2 = rho + N_L2 - gamma * trueIono;

  auto nm_cp = noiseModel::Isotropic::Sigma(1, 0.003);
  auto nm_pr = noiseModel::Isotropic::Sigma(1, 3.0);

  // Build small graph: 1 position, L1 amb, L2 amb, iono, 1 epoch
  NonlinearFactorGraph graph;
  Values initial;

  initial.insert(Key(0), truePos + Point3(10, -5, 3));
  initial.insert(Key(1), N_L1 + 0.5);   // L1 amb
  initial.insert(Key(2), 0.0);           // L1 base amb
  initial.insert(Key(3), N_L2 + 0.3);   // L2 amb
  initial.insert(Key(4), 0.0);           // L2 base amb
  initial.insert(Key(5), 0.0);           // iono

  graph.addPrior<double>(Key(2), 0.0, noiseModel::Isotropic::Sigma(1, 0.001));
  graph.addPrior<double>(Key(4), 0.0, noiseModel::Isotropic::Sigma(1, 0.001));
  graph.addPrior<double>(Key(5), 0.0, noiseModel::Isotropic::Sigma(1, 1.0));

  // DD-PR
  graph.emplace_shared<PseudorangeDDFactor>(
      Key(0), rho + 1.5, sat, satBase, refPos, nm_pr);

  // L1 DD-CP with iono
  graph.emplace_shared<CarrierPhaseDDIonoFactor>(
      Key(0), Key(1), Key(2), Key(5), phi_L1, sat, satBase, refPos, 1.0, nm_cp);

  // L2 DD-CP with iono
  graph.emplace_shared<CarrierPhaseDDIonoFactor>(
      Key(0), Key(3), Key(4), Key(5), phi_L2, sat, satBase, refPos, gamma, nm_cp);

  auto result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  const double est_iono = result.atDouble(Key(5));

  // With iono estimated, iono should be in reasonable range
  EXPECT(std::abs(est_iono - trueIono) < 1.0);
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************
