package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
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
  public static final String CANBUS = "CANivore";

  public static final class Rollers {
    public static final int CAN_ID = 14;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.12, 0.01, 0.2, 0.0, 0.1, 0.0, Optional.empty(), Optional.empty()));

    public static final RollersSpecifications ROLLER_SPECS =
        new RollersSpecifications(16.0 / 24.0, false, Units.inchesToMeters(1.0));

    public static final LinearVelocity MINIMUM_INTAKE_SPEED = MetersPerSecond.of(1.0);
    public static final double DRIVETRAIN_TO_INTAKE_SPEED_FACTOR = 2;
  }

  public static final class Extension {
    public static final int MOTOR_CAN_ID = 15;

    public static final Distance STOWED_POSITION = Meters.zero();
    public static final Distance DEPLOYED_POSITION = Meters.of(0.5);

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.12, 0.05, 0.0, 0.0, 1.0, 0.4, Optional.of(5.0), Optional.of(20.0)));

    public static final ExtensionSpecifications EXTENSION_SPECS =
        new ExtensionSpecifications(15.0, Units.inchesToMeters(1.0), true);
  }

  private IntakeConstants() {}
}
