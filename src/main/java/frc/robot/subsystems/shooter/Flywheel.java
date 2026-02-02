package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIO io;

  @AutoLogOutput(key = "Shooter/TargetFlywheelVelocityRadsPerSec")
  private double targetVelocityRadsPerSec = 0.0;

  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Flywheel/kV", ShooterConstants.Flywheel.getGains().kV());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Flywheel/kS", ShooterConstants.Flywheel.getGains().kS());
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Flywheel/kP", ShooterConstants.Flywheel.getGains().kP());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Flywheel/kD", ShooterConstants.Flywheel.getGains().kD());

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public Command runVolts(double volts) {
    return this.runEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runVelocityRadPerSec(double velocityRadPerSec) {
    return this.runEnd(
        () -> {
          targetVelocityRadsPerSec = velocityRadPerSec;
          io.setVelocity(velocityRadPerSec);
        },
        () -> {
          targetVelocityRadsPerSec = 0.0;
          io.stop();
        });
  }

  public Trigger atTargetVelocity() {
    return new Trigger(() -> inputs.velocityRadsPerSec == targetVelocityRadsPerSec);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            io.setControlConstants(constants[0], constants[1], constants[2], constants[3]),
        kS,
        kV,
        kP,
        kD);
  }
}
