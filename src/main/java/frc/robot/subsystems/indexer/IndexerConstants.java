package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.rollers.RollersSpecifications;
import java.util.Optional;

public final class IndexerConstants {
  public static final String CANBUS = "rio";

  public static final class Serializer {
    public static final int CAN_ID = 17;
    public static final AngularVelocity SPINDEXING_SPEED = RotationsPerSecond.of(5);

    public static final RollersSpecifications ROLLERS_SPECS =
        new RollersSpecifications(5, true, Units.inchesToMeters(8));

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                2.5, 5.0, 0.15, 0.0, 5.0, 0.0, Optional.empty(), Optional.empty()));
  }

  public static final class BallTunneler {
    public static final int CAN_ID = 18;
    public static final AngularVelocity TUNNELING_SPEED = RotationsPerSecond.of(20);

    public static final RollersSpecifications ROLLERS_SPECS =
        new RollersSpecifications(1.0 / 10.0, false, Units.inchesToMeters(13.08 / 2.0));

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.12, 0.01, 0.05, 0.0, 0.5, 0.0, Optional.empty(), Optional.empty()));
  }

  private IndexerConstants() {}
}
