package frc.robot.subsystems.shooter;

import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import java.util.Optional;

public final class ShooterConstants {
  public static final class Flywheel {
    public static final int MAIN_CAN_ID = 0;
    public static final int FOLLOWER_CAN_ID = 1;

    public static final String CANBUS = "rio";

    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kS = 0;

    public static final double kP = 0;
    public static final double kD = 0;

    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 1.0;

    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_RADIUS_METERS = 0.1;
  }

  public static final class Turret {
    public static final int CAN_ID = 2;

    public static final String CANBUS = "rio";

    private static final ControlSystemConstants GAINS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_GAINS,
            new ControlSystemContext(
                1.0, 0.05, 0, 0, 20, 0.5, Optional.of(60.0), Optional.of(100.0)));

    public static ControlSystemContext getGains() {
      return GAINS.getConstants();
    }

    public static final double GEAR_RATIO = 1 / 52.0;
    public static final double MIN_ANGLE_RADS = -190 * (Math.PI / 180);
    public static final double MAX_ANGLE_RADS = 190 * (Math.PI / 180);
  }

  public static final class Hood {
    public static final int CAN_ID = 3;

    public static final String CANBUS = "rio";

    private static final ControlSystemConstants GAINS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_GAINS,
            new ControlSystemContext(
                4.44, 0.1, 0.0, 0.0, 1.0, 0.0, Optional.of(1.0), Optional.of(1.0)));

    public static ControlSystemContext getGains() {
      return GAINS.getConstants();
    }

    public static final double GEAR_RATIO = 1.0;
  }
}
