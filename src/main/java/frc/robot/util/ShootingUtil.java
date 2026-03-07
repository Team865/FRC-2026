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
  public static Angle calculateHoodAngle(double distanceFromHub) {
    // Determined from experimental data and linear regression
    double angleDeg = (7.84808 * distanceFromHub) - 6.31405;

    return Degrees.of(
        MathUtil.clamp(
            angleDeg, ShooterConstants.Hood.MIN_ANGLE_DEG, ShooterConstants.Hood.MAX_ANGLE_DEG));
  }

  public static Angle calculateHoodAngle(Pose2d drivetrainPose, Pose2d targetPose) {
    return calculateHoodAngle(
        targetPose.getTranslation().getDistance(drivetrainPose.getTranslation()));
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

    // Modify the targetPose based on drivetrain speed
    Transform2d targetOffsetVector =
        new Transform2d(
                new Translation2d(
                    -drivetrainFieldOrientedSpeeds.vxMetersPerSecond,
                    -drivetrainFieldOrientedSpeeds.vyMetersPerSecond),
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

  public static AngularVelocity getFlywheelVelocity(double distanceFromHub) {
    return RadiansPerSecond.of(350); // Add more velocities once we get more measurements
  }

  public static double angleRadsBetweenTwoVectors(Translation2d vector1, Translation2d vector2) {
    return Math.acos(vector1.dot(vector2) / (vector1.getNorm() * vector2.getNorm()));
  }

  public static AngularVelocity getAngularVelocityCompensation(
      Pose2d targetPose, Pose2d drivetrainPose, ChassisSpeeds drivetrainSpeeds) {
    Translation2d pointToTargetVector =
        targetPose.getTranslation().minus(drivetrainPose.getTranslation());
    Translation2d linearVelocityVector =
        new Translation2d(drivetrainSpeeds.vxMetersPerSecond, drivetrainSpeeds.vyMetersPerSecond);
    double angleRadsBetweenLookVectorAndVelocity =
        angleRadsBetweenTwoVectors(pointToTargetVector, linearVelocityVector);

    double linearVelocityMagnitudeMeters = linearVelocityVector.getNorm();
    double distanceToTargetMeters = pointToTargetVector.getNorm();

    double linearVelocityCompensationRadsPerSec =
        (Math.sin(angleRadsBetweenLookVectorAndVelocity) * linearVelocityMagnitudeMeters)
            / distanceToTargetMeters;

    return RadiansPerSecond.of(
        drivetrainSpeeds.omegaRadiansPerSecond + linearVelocityCompensationRadsPerSec);
  }
}
