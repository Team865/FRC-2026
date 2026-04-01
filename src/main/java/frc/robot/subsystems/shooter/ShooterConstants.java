package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.pivot.CANcoderSpecifications;
import frc.robot.subsystems.pivot.PivotSpecifications;
import java.util.Optional;

public final class ShooterConstants {
  public static final String CANBUS = "rio";

  public static final class Flywheel {
    public static final int MASTER_CAN_ID = 19;
    public static final int FOLLOWER_CAN_ID = 20;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(
                0.12123, 0.0084668, 0.17012, 0.0, 0.1, 0.0, Optional.empty(), Optional.empty()),
            new ControlSystemContext(
                0.02, 0.01, 0.0, 0.0, 0.1, 0.0, Optional.empty(), Optional.empty()));

    public static final double GEAR_RATIO = 1;
    public static final double WHEEL_RADIUS_METERS = 0.05;

    public static final AngularVelocity SHOOTING_SPEED = RadiansPerSecond.of(350);
    public static final AngularVelocity SETPOINT_TOLERANCE = RotationsPerSecond.of(0.1);
  }

  public static final class Turret {
    public static final int MOTOR_ID = 21;
    public static final int CANCODER_ID = 22;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(
                6.1331, 0, 0.28023, 0.0, 25.0, 0.0, Optional.of(5.0), Optional.of(8.0)),
            new ControlSystemContext(
                1.0, 0.05, 0, 0, 20, 0.5, Optional.of(60.0), Optional.of(100.0)));

    public static final PivotSpecifications PIVOT_SPECS = new PivotSpecifications(52.0, false);
    public static final CANcoderSpecifications CANCODER_SPECS =
        new CANcoderSpecifications(360.0 / 400, false, -0.0373);

    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-190);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(190);
    public static final Angle DEADZONE = Degrees.of(2);
    public static final Angle SHOOTING_TOLERANCE = Degrees.of(5);
    public static final Angle EXTRA_GAIN_TOLERANCE = Degrees.of(0.5);

    // 0.75 rotations / second
    public static final Angle MANUAL_CONTROL_RATE = Rotations.of(0.75).times(0.020);
  }

  public static final class Hood {
    public static final int CAN_ID = 23;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(
                0.24, 0.01, 0.32, 0.0, 300.0, 0.0, Optional.of(5.0), Optional.of(8.0)),
            new ControlSystemContext(
                0.12, 0.001, 0.0, 0.0, 10.0, 0.15, Optional.of(26.0), Optional.of(52.0)));

    public static final PivotSpecifications SPECS = new PivotSpecifications(96.0, true);

    public static final double MIN_ANGLE_DEG = 0;
    public static final double MAX_ANGLE_DEG = 26.5;
    public static final Angle EXTRA_GAIN_TOLERANCE = Degrees.of(0.1);

    // 26.5 degrees / second
    public static final double MANUAL_CONTROL_RATE = 10.0 * 0.020;
  }
}
