package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends FullSubsystem {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();

  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Climber/kG", ClimberConstants.SYSTEM_CONSTANTS.kG);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Climber/kS", ClimberConstants.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Climber/kV", ClimberConstants.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Climber/kA", ClimberConstants.SYSTEM_CONSTANTS.kA);

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Climber/kP", ClimberConstants.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Climber/kD", ClimberConstants.SYSTEM_CONSTANTS.kD);

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Climber/maxVelocity", ClimberConstants.SYSTEM_CONSTANTS.maxVelocity.get());
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Climber/maxAcceleration", ClimberConstants.SYSTEM_CONSTANTS.maxAcceleration.get());

  private final Alert disconnectedAlert =
      new Alert("Climber motor disconnected.", AlertType.kError);
  private Distance target = Meters.of(0);

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;

    climberIO.setControlGains(kG.get(), kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    climberIO.setMotionProfile(maxVelocity.get(), maxAcceleration.get());
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

  public Command stop() {
    return runOnce(climberIO::stop);
  }

  @Override
  public void periodic() {
    // climberIO.updateInputs(climberIOInputs);
    // disconnectedAlert.set(!climberIOInputs.connected);
    // Logger.processInputs("Climber/inputs", climberIOInputs);

    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     (constants) ->
    //         climberIO.setControlGains(
    //             constants[0], constants[1], constants[2], constants[3], constants[4],
    // constants[5]),
    //     kG,
    //     kS,
    //     kV,
    //     kA,
    //     kP,
    //     kD);

    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     (constants) -> climberIO.setMotionProfile(constants[0], constants[1]),
    //     maxVelocity,
    //     maxAcceleration);

    // Logger.recordOutput("Climber/targetMeters", target.in(Meters));
  }
}
