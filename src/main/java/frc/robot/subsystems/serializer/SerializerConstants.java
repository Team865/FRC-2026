package frc.robot.subsystems.serializer;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersConstants;

public class SerializerConstants {

  public static final int CAN_ID = 15;
  public static final RollersConstants ROLLER_CONSTANTS =
      new RollersConstants(
          0.15, // kS
          0.02, // kV
          4.0, // kP
          0.0, // kD
          1 / 10, // gear ratio
          false, // inverted
          40, // current limit in amps
          Units.inchesToMeters(13.08 / 2));
}
