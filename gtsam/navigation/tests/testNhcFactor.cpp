/**
 *  @file   testNhcFactor.cpp
 *  @brief  Unit tests for the non-holonomic constraint (NHC) factors
 *  @date   July 4, 2026
 **/

#include <gtsam/navigation/NhcFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

#include <cmath>
#include <iostream>

using namespace gtsam;
using symbol_shorthand::V;
using symbol_shorthand::X;

static const Key kMount = 100;

// Common measurement inputs.
static const Vector3 kGyro(0.01, -0.02, 0.05);
static const Vector3 kLever(-1.3, 0.1, 0.2);
static const double kWheel = 2.0;
static const SharedNoiseModel kModel =
    noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.05, 0.05));

static const Pose3 kPose(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1, 2, 3));
static const Vector3 kVel(2.0, 0.3, -0.1);
static const Vector3 kPhi(0.02, -0.01, 0.03);

/* ************************************************************************* */
// With identity rotation, world velocity = forward wheel speed, and no
// lever/gyro/mount, the NHC residual is zero.
TEST(NhcFactor, ZeroError) {
  NhcFactor factor(X(0), V(0), Vector3::Zero(), Vector3::Zero(), kWheel, kModel);
  Vector error = factor.evaluateError(Pose3(), Vector3(kWheel, 0.0, 0.0));
  EXPECT(assert_equal(Vector(Vector3::Zero()), error, 1e-9));
}

/* ************************************************************************* */
TEST(NhcFactor, Jacobians) {
  NhcFactor factor(X(0), V(0), kGyro, kLever, kWheel, kModel, kPhi);
  Values values;
  values.insert(X(0), kPose);
  values.insert(V(0), kVel);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

/* ************************************************************************* */
TEST(NhcFactorCalib, ZeroError) {
  NhcFactorCalib factor(X(0), V(0), kMount, Vector3::Zero(), Vector3::Zero(),
                        kWheel, kModel);
  Vector error =
      factor.evaluateError(Pose3(), Vector3(kWheel, 0.0, 0.0), Vector3::Zero());
  EXPECT(assert_equal(Vector(Vector3::Zero()), error, 1e-9));
}

/* ************************************************************************* */
TEST(NhcFactorCalib, Jacobians) {
  NhcFactorCalib factor(X(0), V(0), kMount, kGyro, kLever, kWheel, kModel);
  Values values;
  values.insert(X(0), kPose);
  values.insert(V(0), kVel);
  values.insert(kMount, kPhi);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

/* ************************************************************************* */
// The fixed-angle NhcFactor must match NhcFactorCalib evaluated at that angle.
TEST(NhcFactor, MatchesCalibAtFixedPhi) {
  NhcFactor f2(X(0), V(0), kGyro, kLever, kWheel, kModel, kPhi);
  NhcFactorCalib f3(X(0), V(0), kMount, kGyro, kLever, kWheel, kModel);
  EXPECT(assert_equal(f2.evaluateError(kPose, kVel),
                      f3.evaluateError(kPose, kVel, kPhi), 1e-12));
}

/* ************************************************************************* */
// The forward-axis (roll) component of the mounting angle is unobservable:
// rotating [wheel,0,0] about its own axis is a no-op, so at zero mounting angle
// the first column of H_mountAngle is exactly zero (documented in the header).
TEST(NhcFactorCalib, ForwardAxisUnobservable) {
  NhcFactorCalib factor(X(0), V(0), kMount, kGyro, kLever, kWheel, kModel);
  const Vector3 zero3 = Vector3::Zero();
  Matrix Hp, Hv, Hm;
  factor.evaluateError(kPose, kVel, zero3, Hp, Hv, Hm);
  const Vector3 col0 = Hm.col(0);
  EXPECT(assert_equal(zero3, col0, 1e-9));
}

/* ************************************************************************* */
// SAMPLE 1 — GTSAM-convention functional check.
// Pose3 = wTb (body->world); velocity state is world-frame (NavState convention).
// NHC drives body-frame velocity to [wheel,0,0], so optimizing for the velocity
// with a fixed pose must recover  v_world = wRb * [wheel,0,0].
TEST(NhcFactor, SampleRecoverWorldVelocity) {
  const double wheel = 5.0;
  const Rot3 wRb = Rot3::Yaw(30.0 * M_PI / 180.0);  // 30 deg heading
  const Pose3 pose(wRb, Point3(10, 20, 0));

  const Vector3 zero3 = Vector3::Zero();
  NonlinearFactorGraph graph;
  graph.addPrior(X(0), pose, noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.emplace_shared<NhcFactor>(X(0), V(0), zero3, zero3, wheel,
                                  noiseModel::Isotropic::Sigma(3, 1e-3));

  Values init;
  init.insert(X(0), pose);
  init.insert(V(0), zero3);  // deliberately wrong initial velocity
  const Values result = LevenbergMarquardtOptimizer(graph, init).optimize();

  const Vector3 vWorld = result.at<Vector3>(V(0));
  const Vector3 expected = wRb.matrix() * Vector3(wheel, 0, 0);
  const Vector3 vBody = wRb.matrix().transpose() * vWorld;  // back to body

  std::cout << "[sample] recovered v_world = " << vWorld.transpose()
            << "  (expected " << expected.transpose() << ")\n";
  std::cout << "[sample] body-frame v      = " << vBody.transpose()
            << "  (forward-only [wheel,0,0])\n";

  EXPECT(assert_equal(expected, vWorld, 1e-4));
  EXPECT(assert_equal(Vector3(wheel, 0, 0), vBody, 1e-4));
}

/* ************************************************************************* */
// SAMPLE 2 — NhcFactorCalib recovers a mounting misalignment from many epochs.
// A body that truly drives forward but whose IMU is yaw-misaligned by phi_true
// sees a lateral component; the calib factor should recover phi_true (the
// observable yaw/pitch part; the forward-axis component stays at its prior).
TEST(NhcFactorCalib, SampleRecoverMountAngle) {
  const double wheel = 4.0;
  const Vector3 phiTrue(0.0, 0.0, 0.08);  // ~4.6 deg yaw misalignment
  const Vector3 zero3 = Vector3::Zero();
  const Key kMountEst = 999;

  NonlinearFactorGraph graph;
  Values init;
  init.insert(kMountEst, zero3);
  // loose anchor keeps the unobservable roll DOF finite
  graph.addPrior(kMountEst, zero3, noiseModel::Isotropic::Sigma(3, 1.0));

  // Several epochs at different headings, each with the true world velocity
  // consistent with phiTrue, so the residual is minimized at phi = phiTrue.
  const double headings[] = {0.0, 0.5, 1.0, -0.7, 2.0};
  int i = 0;
  for (double h : headings) {
    const Rot3 wRb = Rot3::Yaw(h);
    const Pose3 pose(wRb, Point3::Zero());
    // body velocity the IMU would see = Expmap(phiTrue) * [wheel,0,0]
    const Vector3 vBody = Rot3::Expmap(phiTrue).rotate(Vector3(wheel, 0, 0));
    const Vector3 vWorld = wRb.matrix() * vBody;

    graph.addPrior(X(i), pose, noiseModel::Isotropic::Sigma(6, 1e-6));
    graph.addPrior(V(i), vWorld, noiseModel::Isotropic::Sigma(3, 1e-6));
    graph.emplace_shared<NhcFactorCalib>(X(i), V(i), kMountEst, Vector3::Zero(),
                                         Vector3::Zero(), wheel,
                                         noiseModel::Isotropic::Sigma(3, 1e-2));
    init.insert(X(i), pose);
    init.insert(V(i), vWorld);
    ++i;
  }

  const Values result = LevenbergMarquardtOptimizer(graph, init).optimize();
  const Vector3 phiEst = result.at<Vector3>(kMountEst);
  std::cout << "[sample] recovered phi = " << phiEst.transpose()
            << "  (true " << phiTrue.transpose() << ")\n";
  EXPECT_DOUBLES_EQUAL(phiTrue.z(), phiEst.z(), 1e-3);  // observable yaw part
}

/* ************************************************************************* */
// SAMPLE 3 — realistic IMU trajectory from a GTSAM ConstantTwistScenario.
// A vehicle drives at 8 m/s and turns left at 0.15 rad/s; the scenario provides
// ground-truth pose(t), body gyro omega_b(t) and nav-frame velocity_n(t) --
// exactly the inputs an IMU would feed the NHC factor. We fix each ground-truth
// pose, feed the scenario gyro + wheel speed, and recover the world velocity.
TEST(NhcFactor, SampleImuScenarioTrajectory) {
  const double speed = 8.0;     // forward body speed [m/s]
  const double yawRate = 0.15;  // left turn [rad/s]
  const ConstantTwistScenario scenario(Vector3(0.0, 0.0, yawRate),
                                       Vector3(speed, 0.0, 0.0));
  const Vector3 zero3 = Vector3::Zero();

  NonlinearFactorGraph graph;
  Values init;
  const double dt = 0.5;
  const int N = 8;
  for (int i = 0; i < N; ++i) {
    const double t = i * dt;
    const Pose3 pose = scenario.pose(t);       // ground-truth wTb
    const Vector3 gyro = scenario.omega_b(t);  // body gyro from the scenario
    graph.addPrior(X(i), pose, noiseModel::Isotropic::Sigma(6, 1e-6));
    graph.emplace_shared<NhcFactor>(X(i), V(i), gyro, zero3, speed,
                                    noiseModel::Isotropic::Sigma(3, 1e-3));
    init.insert(X(i), pose);
    init.insert(V(i), zero3);  // wrong initial velocity
  }

  const Values result = LevenbergMarquardtOptimizer(graph, init).optimize();

  double maxErr = 0.0;
  for (int i = 0; i < N; ++i) {
    const Vector3 vTruth = scenario.velocity_n(i * dt);  // world-frame truth
    const Vector3 vEst = result.at<Vector3>(V(i));
    maxErr = std::max(maxErr, (vEst - vTruth).norm());
    EXPECT(assert_equal(vTruth, vEst, 1e-3));
  }
  std::cout << "[sample] IMU scenario (" << N << " epochs, turning arc): "
            << "max |v_est - v_truth| = " << maxErr << " m/s\n";
  std::cout << "[sample]   t=0   v_truth=" << scenario.velocity_n(0).transpose()
            << "   t=" << (N - 1) * dt
            << " v_truth=" << scenario.velocity_n((N - 1) * dt).transpose()
            << "  (heading rotates with the turn)\n";
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
