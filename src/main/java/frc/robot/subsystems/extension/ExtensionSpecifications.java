package frc.robot.subsystems.extension;

public final record ExtensionSpecifications(
    double gearRatio,
    double drumRadiusMeters,
    boolean clockwisePositive,
    double supplyCurrentLimit,
    double statorCurrentLimit) {
  public ExtensionSpecifications {
    supplyCurrentLimit = Math.min(supplyCurrentLimit, 120.0);
    statorCurrentLimit = Math.min(statorCurrentLimit, 2000.0);
  }

  public ExtensionSpecifications(
      double gearRatio, double drumRadiusMeters, boolean clockwisePositive) {
    this(gearRatio, drumRadiusMeters, clockwisePositive, 120, 2000);
  }
}
