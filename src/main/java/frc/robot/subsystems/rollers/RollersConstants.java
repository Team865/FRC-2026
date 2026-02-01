package frc.robot.subsystems.rollers;

public class RollersConstants {
  public final double kS;
  public final double kV;
  public final double kP;
  public final double kD;

  public final double gearRatio;
  public final boolean inverted;
  public final int currentLimit;
  public final double rollerRadiusMeters;

  public RollersConstants(
      double kS,
      double kV,
      double kP,
      double kD,
      double gearRatio,
      boolean inverted,
      int currentLimit,
      double rollerRadiusMeters) {

    this.kS = kS;
    this.kV = kV;
    this.kP = kP;
    this.kD = kD;
    this.gearRatio = gearRatio;
    this.inverted = inverted;
    this.currentLimit = currentLimit;
    this.rollerRadiusMeters = rollerRadiusMeters;
  }
}
