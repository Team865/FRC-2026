package frc.robot.util.Shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public final class ShotCalculator {
  private ShotCalculator() {}

  public static record ShootingCalculation(
      Translation2d virtualGoal, AngularVelocity flywheelVelocity, Angle pitch, Angle yaw) {}

  /** Distance (m) -> Pitch (deg) */
  private static final int MAX_ITERATIONS = 15;

  private static final double CONVERGENCE_TOLERANCE_METERS = 0.1;

  private static final double kDrag = 1.0;
  private static final LoggedTunableNumber testDragCoefficient =
      new LoggedTunableNumber("Test/DragCoefficient", 1.0);

  public static ShootingCalculation calculateScoringShot(
      Pose2d robotPose, Pose2d targetPose, ChassisSpeeds robotSpeedsFieldOriented) {
    Translation2d origin = robotPose.getTranslation();
    Translation2d goal = targetPose.getTranslation();
    Translation2d relativeTargetSpeedMPS =
        new Translation2d(
            -robotSpeedsFieldOriented.vxMetersPerSecond,
            -robotSpeedsFieldOriented.vyMetersPerSecond);

    Translation2d vectorToGoal = goal.minus(origin);

    if (relativeTargetSpeedMPS.getNorm() == 0) {
      double distanceFromGoalMeters = vectorToGoal.getNorm();

      return new ShootingCalculation(
          targetPose.getTranslation(),
          ShootingMeasurements.getFlywheelVelocity(distanceFromGoalMeters),
          ShootingMeasurements.getScoringPitch(distanceFromGoalMeters),
          vectorToGoal.getAngle().minus(robotPose.getRotation()).getMeasure());
    }

    double timeOfFlightSeconds = 0.0;
    Translation2d virtualGoal = goal;

    double dragCoefficient = testDragCoefficient.get();

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double distanceFromGoalMeters = virtualGoal.getDistance(origin);

      timeOfFlightSeconds = ShootingMeasurements.getScoringToFSeconds(distanceFromGoalMeters);

      // // Apply drag coefficient
      // timeOfFlightSeconds = (1 - Math.exp(-dragCoefficient * timeOfFlightSeconds)) /
      // dragCoefficient;

      Translation2d previousGoal = virtualGoal;
      virtualGoal = goal.plus(relativeTargetSpeedMPS.times(timeOfFlightSeconds));

      if (virtualGoal.getDistance(previousGoal) < CONVERGENCE_TOLERANCE_METERS) break;
    }

    vectorToGoal = virtualGoal.minus(origin);
    double distanceFromGoalMeters = vectorToGoal.getNorm();

    return new ShootingCalculation(
        virtualGoal,
        ShootingMeasurements.getFlywheelVelocity(distanceFromGoalMeters),
        ShootingMeasurements.getScoringPitch(distanceFromGoalMeters),
        vectorToGoal.getAngle().minus(robotPose.getRotation()).getMeasure());
  }

  public static ShootingCalculation calculatePassingShot(
      Pose2d robotPose, ChassisSpeeds robotSpeedsFieldOriented) {

    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d robotSpeedsVector =
        new Translation2d(
            robotSpeedsFieldOriented.vxMetersPerSecond, robotSpeedsFieldOriented.vyMetersPerSecond);
    Translation2d originalGoal =
        FieldConstants.Passing.getPassingTargetPos(!FieldConstants.isOnRightSide(robotPose));
    Translation2d virtualGoal = originalGoal.plus(robotSpeedsVector.times(-0.5));

    double distanceFromTarget = robotTranslation.getDistance(virtualGoal);

    AngularVelocity flywheelVelocity;
    double pitchDegrees;

    if (distanceFromTarget < 7.0) {
      flywheelVelocity = RadiansPerSecond.of(350.0);
      pitchDegrees = 10 * distanceFromTarget - 43.75;
    } else if (distanceFromTarget < 10.0) {
      flywheelVelocity = RadiansPerSecond.of(450.0);
      pitchDegrees = 8.0 * distanceFromTarget - 40;
    } else {
      flywheelVelocity = RadiansPerSecond.of(600.0);
      pitchDegrees = 5.0 * distanceFromTarget - 34;
    }

    pitchDegrees =
        MathUtil.clamp(
            pitchDegrees, ShooterConstants.Hood.MIN_ANGLE_DEG, ShooterConstants.Hood.MAX_ANGLE_DEG);

    return new ShootingCalculation(
        virtualGoal,
        flywheelVelocity,
        Degrees.of(pitchDegrees),
        virtualGoal.minus(robotTranslation).getAngle().minus(robotPose.getRotation()).getMeasure());
  }

  public static double angleRadsBetweenTwoVectors(Translation2d vector1, Translation2d vector2) {
    double vector1Length = MathUtil.applyDeadband(vector1.getNorm(), 0.01);
    double vector2Length = MathUtil.applyDeadband(vector2.getNorm(), 0.01);

    if (vector1Length == 0) {
      if (vector2Length == 0) return 0.0;
      else return vector2.getAngle().getRadians();
    } else if (vector2Length == 0) {
      return vector1.getAngle().getRadians();
    } else {
      return vector2.getAngle().minus(vector1.getAngle()).getRadians();
    }
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
