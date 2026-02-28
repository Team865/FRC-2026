package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.rollers.RollersSpecifications;

public final class IndexerConstants {
  public static final String CANBUS = "CANivore";

  public static final class Serializer {
    public static final int CAN_ID = 17;
    public static final AngularVelocity SPINDEXING_SPEED = RadiansPerSecond.of(15);

    public static final RollersSpecifications ROLLERS_SPECS =
        new RollersSpecifications(10.8, true, Units.inchesToMeters(8));

    private static final double TAU = 1;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(1.1535, 0.1007, 0.45726, 0, 5.0, 0),
            new ControlSystemContext(2.5, 0.1, 0.25, 0.0, 5.0, 0.0));
  }

  public static final class BallTunneler {
    public static final int CAN_ID = 18;
    public static final AngularVelocity TUNNELING_SPEED = RadiansPerSecond.of(900);

    public static final RollersSpecifications ROLLERS_SPECS =
        new RollersSpecifications(1.6875, false, Units.inchesToMeters(13.08 / 2.0));

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            new ControlSystemContext(0.16037, 0.065646, 0.94517, 0, 1.0, 0.0),
            new ControlSystemContext(0.12, 0.01, 0.05, 0.0, 0.5, 0.0));
  }

  private IndexerConstants() {}
}
