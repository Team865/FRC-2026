package frc.robot.subsystems.rollers;

public record RollersSpecifications(
    double gearRatio,
    boolean clockwisePositive,
    double rollerRadiusMeters,
    double supplyCurrentLimit,
    double statorCurrentLimit) {
  public RollersSpecifications {
    supplyCurrentLimit = Math.min(supplyCurrentLimit, 70.0);
    statorCurrentLimit = Math.min(statorCurrentLimit, 120.0);
  }

  public RollersSpecifications(
      double gearRatio, boolean clockwisePositive, double rollerRadiusMeters) {
    this(gearRatio, clockwisePositive, rollerRadiusMeters, 70.0, 120.0);
  }
}
