package frc.robot.subsystems.intake;

import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends Rollers {

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.2);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.01);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 5.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);

  public final Pivot pivot;
  private final LoggedTunableNumber pivot_kS =
      new LoggedTunableNumber("IntakePivot/kS", IntakeConstants.Pivot.getGains().kS());
  private final LoggedTunableNumber pivot_kV =
      new LoggedTunableNumber("IntakePivot/kV", IntakeConstants.Pivot.getGains().kV());
  private final LoggedTunableNumber pivot_kA =
      new LoggedTunableNumber("IntakePivot/kA", IntakeConstants.Pivot.getGains().kA());
  private final LoggedTunableNumber pivot_kP =
      new LoggedTunableNumber("IntakePivot/kP", IntakeConstants.Pivot.getGains().kP());
  private final LoggedTunableNumber pivot_kD =
      new LoggedTunableNumber("IntakePivot/kD", IntakeConstants.Pivot.getGains().kD());

  public Intake(RollersIO rollersIO, PivotIO pivotIO) {
    super("Intake", rollersIO);
    this.pivot = new Pivot("IntakePivot", pivotIO);
  }

  @Override
  public void periodic() {
    int id = hashCode();

    // Update roller control constants if changed
    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);

    // Update pivot control constants if changed
    LoggedTunableNumber.ifChanged(
        id,
        c -> pivot.getIO().setControlConstants(c[0], c[1], c[2], c[3], c[4]),
        pivot_kS,
        pivot_kV,
        pivot_kA,
        pivot_kP,
        pivot_kD);

    super.periodic();
  }
}
