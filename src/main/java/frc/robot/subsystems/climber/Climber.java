package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();

  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Climber/kG", ClimberConstants.getConstants().kG());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Climber/kS", ClimberConstants.getConstants().kS());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Climber/kV", ClimberConstants.getConstants().kV());
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Climber/kA", ClimberConstants.getConstants().kA());

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Climber/kP", ClimberConstants.getConstants().kP());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Climber/kD", ClimberConstants.getConstants().kD());

  private Distance target = Meters.of(0);

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  public Command setPosition(Distance positionTarget) {
    return this.runOnce(
        () -> {
          this.target = positionTarget;
          this.climberIO.setPosition(positionTarget.in(Meters));
        });
  }

  public Command extend() {
    return this.setPosition(ClimberConstants.extendedPosition);
  }

  public Command retract() {
    return this.setPosition(ClimberConstants.retractedPosition);
  }

  public double getPositionMeters() {
    return climberIOInputs.positionMeters;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberIOInputs);
    Logger.processInputs("Climber/inputs", climberIOInputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            climberIO.setControlGains(
                constants[0], constants[1], constants[2], constants[3], constants[4], constants[5]),
        kG,
        kS,
        kV,
        kA,
        kP,
        kD);

    Logger.recordOutput("Climber/targetMeters", target.in(Meters));
  }
}
