package frc.robot.subsystems.balltunneler;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersConstants;

public final class BallTunnelerConstants {

  public static final int CAN_ID = 16;

  public static final RollersConstants ROLLERS =
      new RollersConstants(
          1.0 / 10.0, // gear ratio
          false, // inverted
          40, // current limit (amps)
          Units.inchesToMeters(13.08 / 2.0) // roller radius
          );

  private BallTunnelerConstants() {}
}
