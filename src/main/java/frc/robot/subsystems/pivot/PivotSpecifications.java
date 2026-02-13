package frc.robot.subsystems.pivot;

public record PivotSpecifications(
    double gearRatio,
    boolean clockwisePositive,
    double supplyCurrentLimit,
    double statorCurrentLimit) {
  public PivotSpecifications {
    supplyCurrentLimit = Math.min(supplyCurrentLimit, 70.0);
    statorCurrentLimit = Math.min(statorCurrentLimit, 120.0);
  }

  public PivotSpecifications(double gearRatio, boolean clockwisePositive) {
    this(gearRatio, clockwisePositive, 70.0, 120.0);
  }
}
