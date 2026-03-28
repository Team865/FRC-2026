package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.extension.ExtensionSpecifications;
import frc.robot.subsystems.rollers.RollersSpecifications;
import java.util.Optional;

public final class IntakeConstants {

  public static final class Rollers {
    public static final int CAN_ID = 14;
    public static final String CANBUS = "rio";

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(0.24252, 0.005021, 0.30791, 0.0, 1.0, 0.0),
            new ControlSystemContext(0.12, 0.01, 0.2, 0.0, 0.5, 0.0));

    public static final RollersSpecifications ROLLER_SPECS =
        new RollersSpecifications(2.0, false, Units.inchesToMeters(1), 80.0, 170.0);

    public static final LinearVelocity MINIMUM_INTAKE_SPEED = MetersPerSecond.of(3.5);
    public static final LinearVelocity AGITATING_VELOCITY = MetersPerSecond.of(5);
    public static final double DRIVETRAIN_TO_INTAKE_SPEED_FACTOR = 2;
  }

  public static final class Extension {
    public static final int MOTOR_CAN_ID = 15;
    public static final String CANBUS = "CANivore";

    public static final Distance STOWED_POSITION = Inches.of(3.0);
    public static final Distance PARTIAL_STOWED_POSITION = Inches.of(11.0);
    public static final Distance DEPLOYED_POSITION = Inches.of(12.25);

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(
                2.6, 0.001, 1, 0.0, 3.0, 0, Optional.of(10.0), Optional.of(40.0)),
            new ControlSystemContext(
                0.12, 0.05, 0.0, 0.0, 1.0, 0.4, Optional.of(2.0), Optional.of(20.0)));

    public static final ExtensionSpecifications EXTENSION_SPECS =
        new ExtensionSpecifications(20.0, Units.inchesToMeters(1.0), false);
  }

  private IntakeConstants() {}
}
