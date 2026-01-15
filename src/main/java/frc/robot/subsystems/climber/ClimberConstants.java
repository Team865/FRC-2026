package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public final class ClimberConstants {
  public static final Distance retractedPosition = Inches.of(0);
  public static final Distance extendedPosition =
      Inches.of(30); // Placeholder until the climber is designed

  // Feedforward values taken from last year's elevator for placeholder
  public static final double kG = 0.56;
  public static final double kS = 0.24;
  public static final double kV = 4.44;
  public static final double kA = 0.05;

  public static final double kP = 5.0;
  public static final double kD = 0.0;

  public static final double MAX_VELOCITY = 2.0;
  public static final double MAX_ACCELERATION = 2.0;
}
