package frc.robot.util.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShotCalculator {
  private ShotCalculator() {}

  public static record ShootingCalculation(
      Translation2d virtualGoal, AngularVelocity flywheelVelocity, Angle pitch, Angle yaw) {}

  /** Distance (m) -> Pitch (deg) */
  private static final int MAX_ITERATIONS = 15;

  private static final double CONVERGENCE_TOLERANCE_METERS = 0.1;

  static {
    // Add measurements here
  }

  public static ShootingCalculation calculate(
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
          vectorToGoal.getAngle().getMeasure(),
          ShootingMeasurements.getScoringPitch(distanceFromGoalMeters));
    }

    double timeOfFlightSeconds;
    Translation2d virtualGoal = goal;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double distanceFromGoalMeters = virtualGoal.getDistance(origin);

      timeOfFlightSeconds = ShootingMeasurements.getScoringToFSeconds(distanceFromGoalMeters);
      Translation2d previousGoal = virtualGoal;
      virtualGoal = goal.plus(relativeTargetSpeedMPS.times(timeOfFlightSeconds));

      if (virtualGoal.getDistance(previousGoal) < CONVERGENCE_TOLERANCE_METERS) break;
    }

    vectorToGoal = virtualGoal.minus(origin);
    double distanceFromGoalMeters = vectorToGoal.getNorm();

    return new ShootingCalculation(
        virtualGoal,
        ShootingMeasurements.getFlywheelVelocity(distanceFromGoalMeters),
        vectorToGoal.getAngle().getMeasure(),
        ShootingMeasurements.getScoringPitch(distanceFromGoalMeters));
  }
}
