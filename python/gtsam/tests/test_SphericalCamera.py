"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SphericalCamera unit tests.
Author: auto-generated
"""
import unittest

import numpy as np

import gtsam
from gtsam import Point3, Pose3, Rot3, SphericalCamera, Unit3
from gtsam.utils.test_case import GtsamTestCase

# Camera pose: Rot3 with diagonal (1, -1, -1), translation (0, 0, 0.5)
pose = Pose3(Rot3(np.diag([1., -1., -1.])), Point3(0, 0, 0.5))
camera = SphericalCamera(pose)

# Test points
point1 = Point3(-0.08, -0.08, 0.0)
point2 = Point3(-0.08, 0.08, 0.0)
point3 = Point3(0.08, 0.08, 0.0)
point4 = Point3(0.08, -0.08, 0.0)

# Manually computed bearing vectors (from Matlab)
bearing1 = Unit3(np.array([-0.156054862928174, 0.156054862928174, 0.975342893301088]))
bearing2 = Unit3(np.array([-0.156054862928174, -0.156054862928174, 0.975342893301088]))
bearing3 = Unit3(np.array([0.156054862928174, -0.156054862928174, 0.975342893301088]))
bearing4 = Unit3(np.array([0.156054862928174, 0.156054862928174, 0.975342893301088]))

depth = 0.512640224719052


class TestSphericalCamera(GtsamTestCase):

    def test_constructor(self):
        """Test construction from Pose3 and pose() accessor."""
        self.gtsamAssertEquals(camera.pose(), pose)

    def test_project(self):
        """Test projection of 4 points to bearing vectors."""
        self.gtsamAssertEquals(camera.project(point1), bearing1)
        self.gtsamAssertEquals(camera.project(point2), bearing2)
        self.gtsamAssertEquals(camera.project(point3), bearing3)
        self.gtsamAssertEquals(camera.project(point4), bearing4)

    def test_backproject(self):
        """Test back-projection from bearing + depth to 3D point."""
        self.gtsamAssertEquals(camera.backproject(bearing1, depth), point1)
        self.gtsamAssertEquals(camera.backproject(bearing2, depth), point2)
        self.gtsamAssertEquals(camera.backproject(bearing3, depth), point3)
        self.gtsamAssertEquals(camera.backproject(bearing4, depth), point4)

    def test_project_jacobian(self):
        """Test project() Jacobians against numerical derivatives."""
        Dpose = np.zeros((2, 6), order='F')
        Dpoint = np.zeros((2, 3), order='F')
        result = camera.project(point1, Dpose, Dpoint)
        self.gtsamAssertEquals(result, bearing1)

        # Numerical Jacobians via finite differences
        delta = 1e-5

        # Jacobian w.r.t. pose
        numerical_Dpose = np.zeros((2, 6))
        for i in range(6):
            d = np.zeros(6)
            d[i] = delta
            cam_plus = SphericalCamera(pose.retract(d))
            cam_minus = SphericalCamera(pose.retract(-d))
            bearing_plus = cam_plus.project(point1)
            bearing_minus = cam_minus.project(point1)
            numerical_Dpose[:, i] = bearing_minus.localCoordinates(bearing_plus) / (2.0 * delta)

        # Jacobian w.r.t. point
        numerical_Dpoint = np.zeros((2, 3))
        for i in range(3):
            d = np.zeros(3)
            d[i] = delta
            bearing_plus = camera.project(point1 + d)
            bearing_minus = camera.project(point1 - d)
            numerical_Dpoint[:, i] = bearing_minus.localCoordinates(bearing_plus) / (2.0 * delta)

        np.testing.assert_allclose(Dpose, numerical_Dpose, atol=1e-5)
        np.testing.assert_allclose(Dpoint, numerical_Dpoint, atol=1e-5)

    def test_reprojection_error(self):
        """Test that reprojection error is zero at ground truth."""
        Dpose = np.zeros((2, 6), order='F')
        Dpoint = np.zeros((2, 3), order='F')
        result = camera.reprojectionError(point1, bearing1, Dpose, Dpoint)
        np.testing.assert_allclose(result, np.zeros(2), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
