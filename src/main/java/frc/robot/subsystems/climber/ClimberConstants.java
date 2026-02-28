package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import java.util.Optional;

public final class ClimberConstants {
  public static final int CAN_ID = 24;
  public static final String CANBUS = "CANivore";

  public static final Distance retractedPosition = Inches.of(0);
  public static final Distance extendedPosition =
      Inches.of(0); // Placeholder until the climber is designed

  public static final ControlSystemConstants SYSTEM_CONSTANTS =
      new ControlSystemConstants(
          new ControlSystemContext(0.12, 0, 0.2, 0, 1, 0, Optional.of(2.0), Optional.of(500.0)),
          new ControlSystemContext(
              4.44, 0.05, 0.24, 0.56, 5.0, 0.0, Optional.of(2.0), Optional.of(2.0)));

  public static final double DRUM_RADIUS_METERS = 0.5;
  public static final double GEAR_RATIO = 400; // Sensor to mechanism (Reduction)
}
