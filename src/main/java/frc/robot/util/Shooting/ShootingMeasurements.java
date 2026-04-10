package frc.robot.util.Shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.ShooterConstants;

public final class ShootingMeasurements {
  private ShootingMeasurements() {}

  private static class ShootingRange {
    public final InterpolatingTreeMap<Double, Double> distanceToPitchDegrees =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    public final InterpolatingTreeMap<Double, Double> distanceToToFSeconds =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    public final AngularVelocity flywheelVelocity;

    public ShootingRange(AngularVelocity flywheelVelocity) {
      this.flywheelVelocity = flywheelVelocity;
    }

    public void addMeasurement(double distanceMeters, double pitchDegrees, double tofSeconds) {
      distanceToPitchDegrees.put(distanceMeters, pitchDegrees);
      distanceToToFSeconds.put(distanceMeters, tofSeconds);
    }
  }

  private static final double SCORING_SHORT_MAX_THRESHOLD_METERS =
      // 100.0;
      3.0;
  private static final double SCORING_MEDIUM_MAX_THRESHOLD_METERS =
      // 100.0;
      6.0;

  private static final ShootingRange scoringShortRange =
      new ShootingRange(RadiansPerSecond.of(300.0));
  private static final ShootingRange scoringMediumRange =
      new ShootingRange(RadiansPerSecond.of(400.0));
  private static final ShootingRange scoringLongRange =
      new ShootingRange(RadiansPerSecond.of(450.0));

  private static long lastAttemptedRangeChangeTimeMillis = 0;
  private static Range currentZone = Range.SHORT;
  private static Range lastWantedZone = Range.SHORT;

  static {
    // Add measurements
    // scoringShortRange.addMeasurement(1.0, 0, 1.575);
    // scoringShortRange.addMeasurement(2.88, 10.5, 1.408333);
    // scoringShortRange.addMeasurement(2.22, 5, 1.39166666);
    // scoringShortRange.addMeasurement(4.505, 18, 1.36);
    // scoringShortRange.addMeasurement(3.0, 10.5, 1.34);
    // scoringShortRange.addMeasurement(1.71, 3, 1.33);
    // scoringShortRange.addMeasurement(3.51, 15, 1.32);
    // scoringShortRange.addMeasurement(4.0, 16.5, 1.3);
    scoringShortRange.addMeasurement(1.16, 3, 1.2);
    scoringShortRange.addMeasurement(1.55, 7, 1.185);
    scoringShortRange.addMeasurement(2, 11, 1.16);
    scoringShortRange.addMeasurement(2.49, 15, 1.14);
    scoringShortRange.addMeasurement(3, 22, 1.04);

    scoringMediumRange.addMeasurement(1.55, 0, 1.73793);
    scoringMediumRange.addMeasurement(5.35, 19, 1.7);
    scoringMediumRange.addMeasurement(3.02, 6.5, 1.6708333);
    scoringMediumRange.addMeasurement(2.5, 3, 1.63);
    scoringMediumRange.addMeasurement(4.08, 11.5, 1.6194444);
    scoringMediumRange.addMeasurement(4.5, 13, 1.595);
    scoringMediumRange.addMeasurement(4.9, 15, 1.572222);
    scoringMediumRange.addMeasurement(3.5, 8, 1.521794872);
    scoringMediumRange.addMeasurement(2.0, 2, 1.47);

    scoringLongRange.addMeasurement(5.0, 12, 1.93);
    scoringLongRange.addMeasurement(4.0, 8, 1.8638888);
    scoringLongRange.addMeasurement(3.0, 3.7, 1.825);
    scoringLongRange.addMeasurement(4.5, 9.5, 1.74583333);
    scoringLongRange.addMeasurement(3.5, 5, 1.6733333);
    scoringLongRange.addMeasurement(5.5, 14, 1.62);

    // Interpolated values found via linear regression
    scoringShortRange.addMeasurement(0.0, -6.11861, 1.5681);
    scoringShortRange.addMeasurement(100.0, 552.09907, -5.0886);

    scoringMediumRange.addMeasurement(0.0, -8.17055, 1.8332);
    scoringMediumRange.addMeasurement(100.0, 474.42475, -4.47762);

    scoringLongRange.addMeasurement(0.0, -9.27143, 2.31083);
    scoringLongRange.addMeasurement(100.0, 413.58571, -10.26536);
  }

  public static double getScoringToFSeconds(double distanceMeters) {
    if (distanceMeters < SCORING_SHORT_MAX_THRESHOLD_METERS) {
      return scoringShortRange.distanceToToFSeconds.get(distanceMeters);
    } else if (distanceMeters < SCORING_MEDIUM_MAX_THRESHOLD_METERS) {
      return scoringMediumRange.distanceToToFSeconds.get(distanceMeters);
    } else {
      return scoringLongRange.distanceToToFSeconds.get(distanceMeters);
    }
  }

  public static Angle getScoringPitch(double distanceMeters) {
    double pitchDegrees;

    if (distanceMeters < SCORING_SHORT_MAX_THRESHOLD_METERS) {
      pitchDegrees = scoringShortRange.distanceToPitchDegrees.get(distanceMeters);
    } else if (distanceMeters < SCORING_MEDIUM_MAX_THRESHOLD_METERS) {
      pitchDegrees = scoringMediumRange.distanceToPitchDegrees.get(distanceMeters);
    } else {
      pitchDegrees = scoringLongRange.distanceToPitchDegrees.get(distanceMeters);
    }

    pitchDegrees =
        MathUtil.clamp(
            pitchDegrees, ShooterConstants.Hood.MIN_ANGLE_DEG, ShooterConstants.Hood.MAX_ANGLE_DEG);

    return Degrees.of(pitchDegrees);
  }

  public static AngularVelocity getFlywheelVelocity(double distanceMeters) {
    if (distanceMeters < SCORING_SHORT_MAX_THRESHOLD_METERS) {
      return scoringShortRange.flywheelVelocity;
    } else if (distanceMeters < SCORING_MEDIUM_MAX_THRESHOLD_METERS) {
      return scoringMediumRange.flywheelVelocity;
    } else {
      return scoringLongRange.flywheelVelocity;
    }
  }
}
