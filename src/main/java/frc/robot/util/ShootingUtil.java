package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShootingUtil {
  private ShootingUtil() {}

  public static record ShootingCalculation(AngularVelocity flywheelSpeed, Angle yaw, Angle pitch) {}

  /** Distance (m) -> Pitch (deg) */
  private static final InterpolatingTreeMap<Double, Double> distanceToPitch350radsps =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

  private static final InterpolatingTreeMap<Double, Double> distanceToToF350radps =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final int numIterations = 5;

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
      return new ShootingCalculation(
          RadiansPerSecond.of(350.0),
          vectorToGoal.getAngle().getMeasure(),
          Degrees.of(distanceToPitch350radsps.get(vectorToGoal.getNorm())));
    }

    double timeOfFlightSeconds;
    Translation2d virtualGoal = goal;

    for (int i = 0; i < numIterations; i++) {
      double distanceFromGoalMeters = virtualGoal.getDistance(origin);

      timeOfFlightSeconds = distanceToToF350radps.get(distanceFromGoalMeters);
      virtualGoal = goal.plus(relativeTargetSpeedMPS.times(timeOfFlightSeconds));
    }

    vectorToGoal = virtualGoal.minus(origin);

    return new ShootingCalculation(
        RadiansPerSecond.of(350.0),
        Degrees.of(vectorToGoal.getAngle().getDegrees()),
        Degrees.of(distanceToPitch350radsps.get(vectorToGoal.getNorm())));
  }
}
