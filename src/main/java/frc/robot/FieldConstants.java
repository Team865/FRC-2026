// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  // Built in apriltag layout
  // Default is welded, the field type used in Ontario district
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Field dimensions (meters)
  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  // tag info
  public static final int aprilTagCount = aprilTagLayout.getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final List<Integer> hubTagIds =
      Arrays.asList(18, 19, 20, 21, 24, 25, 26, 27); // tags for blue hub
  // Scoring element positions
  public static final Pose2d allianceHubPose =
      new Pose2d(
          Units.inchesToMeters(181.56),
          fieldWidth / 2.0,
          Rotation2d.fromDegrees(180)); // Towards origin

  public static final Pose2d allianceRightClimbPose =
      new Pose2d(
          Units.inchesToMeters(44.88),
          Units.inchesToMeters(115.08),
          Rotation2d.fromDegrees(0)); // Away from origin
  public static final Pose2d allianceLeftClimbPose =
      new Pose2d(
          Units.inchesToMeters(38.62),
          Units.inchesToMeters(180.75),
          Rotation2d.fromDegrees(180)); // Away from origin
}
