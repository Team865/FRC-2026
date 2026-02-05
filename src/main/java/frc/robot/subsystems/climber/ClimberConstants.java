package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import java.util.Optional;

public final class ClimberConstants {
  public static final int CAN_ID = 10;
  public static final String CANBUS = "rio";

  public static final Distance retractedPosition = Inches.of(0);
  public static final Distance extendedPosition =
      Inches.of(5); // Placeholder until the climber is designed

  private static final ControlSystemConstants GAINS =
      new ControlSystemConstants(
          ControlSystemConstants.EMPTY_GAINS,
          new ControlSystemContext(
              4.44, 0.05, 0.24, 0.56, 5.0, 0.0, Optional.of(2.0), Optional.of(2.0)));

  public static ControlSystemContext getConstants() {
    return GAINS.getConstants();
  }

  public static final double DRUM_RADIUS_METERS = 0.5;
  public static final double GEAR_RATIO = 400; // Sensor to mechanism (Reduction)
}
