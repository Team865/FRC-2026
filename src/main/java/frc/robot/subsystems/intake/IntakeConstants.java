package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersConstants;

public final class IntakeConstants {

  public static final int CAN_ID = 14;

  public static final RollersConstants ROLLER_CONSTANTS =
      new RollersConstants(
          0.2, // kS
          0.01, // kV
          5.0, // kP
          0.0, // kD
          16.0 / 24.0, // gear ratio
          false, // inverted
          40, // current limit in amps
          Units.inchesToMeters(1.0) // Radius
          );

  private IntakeConstants() {}
}
