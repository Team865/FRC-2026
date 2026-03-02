package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.util.Units;
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
                0.12, 0.0, 0.4, 0.0, 0.5, 0.004, Optional.empty(), Optional.empty()),
            new ControlSystemContext(
                0.12, 0.05, 0.0, 0.0, 0.5, 0.0, Optional.empty(), Optional.empty()));

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
                6.1331, 0, 0.28023, 0.0, 50, 0.0, Optional.of(2.0), Optional.of(5.0)),
            new ControlSystemContext(
                1.0, 0.05, 0, 0, 20, 0.5, Optional.of(60.0), Optional.of(100.0)));

    public static final PivotSpecifications PIVOT_SPECS = new PivotSpecifications(52.0, false);
    public static final CANcoderSpecifications CANCODER_SPECS =
        new CANcoderSpecifications(360.0 / 400, false, 0.241699);

    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-200);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(200);
  }

  public static final class Hood {
    public static final int CAN_ID = 23;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(
                0.24, 0.01, 0.32, 0.0, 600.0, 0.0, Optional.of(5.0), Optional.of(5000.0)),
            new ControlSystemContext(
                4.44, 0.1, 0.0, 0.0, 1.0, 0.0, Optional.of(1.0), Optional.of(1.0)));

    public static final PivotSpecifications SPECS = new PivotSpecifications(96.0, true);

    public static final double MIN_ANGLE_DEG = 0;
    public static final double MAX_ANGLE_DEG = 26.5;
  }
}
