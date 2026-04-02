package frc.robot.util.Shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

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
  }

  private static final double SCORING_SHORT_MAX_THRESHOLD_METERS = 2.5;

  private static final ShootingRange scoringShortRange =
      new ShootingRange(RadiansPerSecond.of(350.0));
  private static final ShootingRange scoringMediumRange =
      new ShootingRange(RadiansPerSecond.of(400.0));

  static {
    // Add measurements
  }

  public static double getScoringToFSeconds(double distanceMeters) {
    if (distanceMeters < SCORING_SHORT_MAX_THRESHOLD_METERS) {
      return scoringShortRange.distanceToToFSeconds.get(distanceMeters);
    } else {
      return scoringMediumRange.distanceToToFSeconds.get(distanceMeters);
    }
  }

  public static Angle getScoringPitch(double distanceMeters) {
    double pitchDegrees;

    if (distanceMeters < SCORING_SHORT_MAX_THRESHOLD_METERS) {
      pitchDegrees = scoringShortRange.distanceToPitchDegrees.get(distanceMeters);
    } else {
      pitchDegrees = scoringMediumRange.distanceToPitchDegrees.get(distanceMeters);
    }

    return Degrees.of(pitchDegrees);
  }

  public static AngularVelocity getFlywheelVelocity(double distanceMeters) {
    if (distanceMeters < SCORING_SHORT_MAX_THRESHOLD_METERS) {
      return scoringShortRange.flywheelVelocity;
    } else {
      return scoringMediumRange.flywheelVelocity;
    }
  }
}
