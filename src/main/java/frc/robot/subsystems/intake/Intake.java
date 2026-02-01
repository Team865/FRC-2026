package frc.robot.subsystems.intake;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends Rollers {

  // Tunable PID / feedforward numbers for live tuning
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake/kS", IntakeConstants.ROLLER_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/kV", IntakeConstants.ROLLER_CONSTANTS.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/kP", IntakeConstants.ROLLER_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/kD", IntakeConstants.ROLLER_CONSTANTS.kD);

  public Intake(RollersIO io) {
    super("Intake", io, IntakeConstants.ROLLER_CONSTANTS);

    // Apply initial PID constants to the IO
    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update PID / feedforward constants if changed on the dashboard
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            getIO().setControlConstants(constants[0], constants[1], constants[2], constants[3]),
        kS,
        kV,
        kP,
        kD);
  }
}
