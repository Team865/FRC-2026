package frc.robot.subsystems.serializer;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersConstants;

public class SerializerConstants {

  public static final int CAN_ID = 15;
  public static final RollersConstants ROLLER_CONSTANTS =
      new RollersConstants(
          0.15, // kS
          0.02, // kV
          0.5, // kP
          0.0, // kD
          10, // gear ratio
          true, // inverted
          40, // current limit in amps
          Units.inchesToMeters(13.08 / 2));
}
