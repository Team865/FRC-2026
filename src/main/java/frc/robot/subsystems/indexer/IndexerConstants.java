package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.rollers.RollersSpecifications;

public final class IndexerConstants {
  public static final String CANBUS = "CANivore";
  public static final double antiStallTimeoutSeconds = 5.0;

  public static final class Serializer {
    public static final int CAN_ID = 17;

    public static AngularVelocity getSerializingSpeed() {
      return DriverStation.isAutonomous() ? RotationsPerSecond.of(1.1) : RotationsPerSecond.of(1.0);
    }

    public static final RollersSpecifications ROLLERS_SPECS =
        new RollersSpecifications(64 / 3, true, Units.inchesToMeters(8), 50, 200);

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(2.1862, 0.27345, 0.8, 0, 5, 0),
            new ControlSystemContext(0.02, 0.01, 0.0, 0.0, 0.5, 0.0));
  }

  public static final class BallTunneler {
    public static final int CAN_ID = 18;
    public static final AngularVelocity TUNNELING_SPEED = RadiansPerSecond.of(246.0);

    public static final RollersSpecifications ROLLERS_SPECS =
        new RollersSpecifications(1.6875, false, Units.inchesToMeters(13.08 / 2.0), 50, 200);

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(0.21301, 0.0039799, 0.39976, 0, 1.5, 0.0),
            new ControlSystemContext(0.02, 0.01, 0, 0.0, 0.5, 0.0));
  }

  private IndexerConstants() {}
}
