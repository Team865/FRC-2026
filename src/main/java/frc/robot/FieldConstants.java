// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
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
      Arrays.asList(18, 19, 20, 21, 24, 25, 26, 27, 23); // tags for blue hub

  // Scoring element positions
  public static final Pose2d allianceHubPoseBlue =
      new Pose2d(
          Units.inchesToMeters(181.56),
          Units.inchesToMeters(158.845),
          Rotation2d.fromDegrees(180)); // Towards origin
  public static final Pose2d allianceHubPoseRed =
      new Pose2d(
          Units.inchesToMeters(468.56),
          FieldConstants.fieldWidth / 2,
          Rotation2d.fromDegrees(0)); // Away from origin

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

  public static final class Passing {
    private static final double redBumpLineXPos = Units.inchesToMeters(651.22 - 182.11);
    private static final double blueBumpLineXPos = Units.inchesToMeters(182.11);

    private static final Pose2d redLeftCorner =
        new Pose2d(Units.inchesToMeters(651.22), Units.inchesToMeters(20.0), Rotation2d.kZero);
    private static final Pose2d blueLeftCorner =
        new Pose2d(0.0, Units.inchesToMeters(293.69), Rotation2d.kZero);

    private static final Pose2d redRightCorner =
        new Pose2d(Units.inchesToMeters(651.22), Units.inchesToMeters(293.69), Rotation2d.kZero);
    private static final Pose2d blueRightCorner =
        new Pose2d(0.0, Units.inchesToMeters(20.0), Rotation2d.kZero);

    public static double getBumpLineXPos() {
      return AllianceFlipUtil.shouldFlip() ? redBumpLineXPos : blueBumpLineXPos;
    }

    public static boolean shouldBePassing(Pose2d robotPose) {
      return (AllianceFlipUtil.shouldFlip())
          ? robotPose.getX() < redBumpLineXPos
          : robotPose.getX() > blueBumpLineXPos;
    }

    public static Pose2d getLeftCorner() {
      return AllianceFlipUtil.shouldFlip() ? redLeftCorner : blueLeftCorner;
    }

    public static Pose2d getRightCorner() {
      return AllianceFlipUtil.shouldFlip() ? redRightCorner : blueRightCorner;
    }
  }

  public static boolean isOnRightSide(Pose2d robotPose) {
    if (AllianceFlipUtil.shouldFlip()) return robotPose.getY() < fieldWidth / 2;
    else return robotPose.getY() > fieldWidth / 2;
  }

  public static final class Lockout {
    public static final double thresholdMeters = 1.0;

    /* Returns the coordinates of the lockout zone as a pair of Translation2d's, where the first is the minimum and second the maximum. */
    public static Pair<Translation2d, Translation2d> getZone() {
      double robotCenterToEdgeMeters = Units.inchesToMeters(27.5 / 2);
      Translation2d minOffset = new Translation2d(robotCenterToEdgeMeters, robotCenterToEdgeMeters);
      Translation2d maxOffset =
          new Translation2d(-robotCenterToEdgeMeters, -robotCenterToEdgeMeters);

      return AllianceFlipUtil.shouldFlip()
          ? new Pair<>(
              new Translation2d(fieldLength / 2.0, 0.0).plus(minOffset),
              new Translation2d(fieldLength, fieldWidth).plus(maxOffset))
          : new Pair<>(
              Translation2d.kZero.plus(minOffset),
              new Translation2d(fieldLength / 2.0, fieldWidth).plus(maxOffset));
    }
  }
}
