package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShootingUtil {
  public static Angle calculateHoodAngle(double distanceFromTargetMeters) {
    // Determined from experimental data and linear regression
    double angleDeg; // = (7.84808 * distanceFromTargetMeters) - 6.31405;

    if (distanceFromTargetMeters < 2.5) {
      angleDeg = (7.84808 * distanceFromTargetMeters) - 6.31405;
    } else {
      angleDeg = (7.0125 * distanceFromTargetMeters) - 8.6826;
    }

    return Degrees.of(
        MathUtil.clamp(
            angleDeg, ShooterConstants.Hood.MIN_ANGLE_DEG, ShooterConstants.Hood.MAX_ANGLE_DEG));
  }

  public static Angle calculateTurretRelativeAngle(Pose2d drivetrainPose, Pose2d targetPose) {
    Logger.recordOutput("Turret/Target Pose", targetPose);

    Rotation2d driveHeading = drivetrainPose.getRotation();
    Translation2d driveToHubVector =
        targetPose.getTranslation().minus(drivetrainPose.getTranslation());
    Rotation2d pointToHubRotation =
        new Rotation2d(driveToHubVector.getX(), driveToHubVector.getY());

    return pointToHubRotation.minus(driveHeading).getMeasure();
  }

  public static Pose2d correctTargetPoseWhileMoving(
      Pose2d targetPose, ChassisSpeeds drivetrainFieldOrientedSpeeds, double correctionFactor) {

    if (AllianceFlipUtil.shouldFlip())
      drivetrainFieldOrientedSpeeds = drivetrainFieldOrientedSpeeds.times(-1);

    // Modify the targetPose based on drivetrain speed
    Transform2d targetOffsetVector =
        new Transform2d(
                new Translation2d(
                    drivetrainFieldOrientedSpeeds.vxMetersPerSecond,
                    drivetrainFieldOrientedSpeeds.vyMetersPerSecond),
                Rotation2d.kZero)
            .times(correctionFactor);

    return targetPose.plus(targetOffsetVector);
  }

  public static Pose2d correctTargetPoseWhileMoving(
      Pose2d targetPose, ChassisSpeeds drivetrainFieldOrientedSpeeds) {
    double correctionFactor = 0.85;

    return correctTargetPoseWhileMoving(
        targetPose, drivetrainFieldOrientedSpeeds, correctionFactor);
  }

  public static AngularVelocity getFlywheelVelocity(double distanceFromTargetMeters) {
    if (distanceFromTargetMeters < 2.5) {
      return RadiansPerSecond.of(350);
    } else {
      return RadiansPerSecond.of(400);
    }

    // return RadiansPerSecond.of(150); // Add more velocities once we get more measurements
  }

  public static double angleRadsBetweenTwoVectors(Translation2d vector1, Translation2d vector2) {
    return vector2.getAngle().minus(vector1.getAngle()).getRadians();
  }

  public static AngularVelocity getAngularVelocityCompensation(
      Pose2d drivetrainPose, Pose2d targetPose, ChassisSpeeds drivetrainSpeeds) {
    Translation2d pointToTargetVector =
        targetPose.getTranslation().minus(drivetrainPose.getTranslation());
    Translation2d linearVelocityVector =
        new Translation2d(drivetrainSpeeds.vxMetersPerSecond, drivetrainSpeeds.vyMetersPerSecond);
    double drivetrainSpeedMPS = linearVelocityVector.getNorm();

    double linearVelocityCompensationRadsPerSec;

    if (drivetrainSpeedMPS == 0) {
      linearVelocityCompensationRadsPerSec = 0.0;
    } else {
      double angleRadsBetweenLookVectorAndVelocity =
          angleRadsBetweenTwoVectors(pointToTargetVector, linearVelocityVector);
      double linearVelocityMagnitudeMeters = drivetrainSpeedMPS;
      double distanceToTargetMeters = pointToTargetVector.getNorm();

      linearVelocityCompensationRadsPerSec =
          (Math.sin(angleRadsBetweenLookVectorAndVelocity) * linearVelocityMagnitudeMeters)
              / distanceToTargetMeters;
    }

    return RadiansPerSecond.of(
        -drivetrainSpeeds.omegaRadiansPerSecond + linearVelocityCompensationRadsPerSec);
  }
}
