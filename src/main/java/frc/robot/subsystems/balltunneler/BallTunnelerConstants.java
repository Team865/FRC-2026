package frc.robot.subsystems.balltunneler;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersConstants;

public class BallTunnelerConstants {

  public static final int CAN_ID = 16;

  public static final RollersConstants ROLLER_CONSTANTS =
      new RollersConstants(
          0.15, // kS
          0.02, // kV
          4.5, // kP
          0.0, // kD
          1 / 10, // gear ratio
          false, // inverted
          40, // current limit
          Units.inchesToMeters(13.08 / 2));
}
