// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = FieldConstants.aprilTagLayout;

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FrontCamera";
  public static String camera1Name = "BackCamera";
  public static String camera2Name = "TurretCamera";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.3, 0.0, 0.127, new Rotation3d(0, Units.degreesToRadians(-30), 90));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          0.3,
          0.0,
          0.5,
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-90)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          0.3,
          0.0,
          Units.inchesToMeters(25.25),
          new Rotation3d(0, Units.degreesToRadians(-200), 0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Front
        1.0, // Back
        1.0 // Turret
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
