package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem implements SysIdTestable {
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  public final FlywheelIO io;

  @AutoLogOutput(key = "Shooter/TargetFlywheelVelocityRadsPerSec")
  private AngularVelocity targetVelocity = RotationsPerSecond.zero();

  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Flywheel/kV", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Flywheel/kA", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Flywheel/kS", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Flywheel/kP", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Flywheel/kD", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kD);

  private final Alert masterDisconnectedAlert =
      new Alert("Flywheel master motor disconnected.", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Flywheel follower motor disconnected.", AlertType.kError);

  private final SysIdRoutine sysIdRoutine;

  public Flywheel(FlywheelIO io) {
    this.io = io;

    io.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    sysIdRoutine =
        new SysIdBuilder(this, io::setVolts)
            .withDynamicStepVoltage(6.0)
            .withQuasistaticRampRate(1.0)
            .build();
  }

  public Command setVolts(double volts) {
    return this.runOnce(() -> io.setVolts(volts));
  }

  public Command setVelocity(AngularVelocity velocity) {
    return this.runOnce(
        () -> {
          targetVelocity = velocity;
          io.setVelocity(velocity);
        });
  }

  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return this.runOnce(
        () -> {
          targetVelocity = velocitySupplier.get();
          io.setVelocity(targetVelocity);
        });
  }

  public Command runVolts(double volts) {
    return this.runEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runVolts(DoubleSupplier voltsSupplier) {
    return this.runEnd(() -> io.setVolts(voltsSupplier.getAsDouble()), () -> io.setVolts(0));
  }

  public Command runVelocity(AngularVelocity velocity) {
    return this.runEnd(
        () -> {
          targetVelocity = velocity;
          io.setVelocity(targetVelocity);
        },
        () -> {
          targetVelocity = RotationsPerSecond.zero();
          io.stop();
        });
  }

  public Command runVelocityWithoutStopping(Supplier<AngularVelocity> velocitySupplier) {
    return this.run(
        () -> {
          targetVelocity = velocitySupplier.get();
          io.setVelocity(targetVelocity);
        });
  }

  public AngularVelocity getAngularVelocity() {
    return inputs.masterVelocity;
  }

  public Command runVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return this.runEnd(
        () -> {
          targetVelocity = velocitySupplier.get();
          io.setVelocity(targetVelocity);
        },
        () -> {
          targetVelocity = RotationsPerSecond.zero();
          io.stop();
        });
  }

  public Command stop() {
    return this.runOnce(io::stop);
  }

  public Trigger atTargetVelocity() {
    return new Trigger(() -> isAtSetpoint(targetVelocity));
  }

  public boolean isAtSetpoint(AngularVelocity setpoint) {
    return inputs.masterVelocity.isNear(setpoint, ShooterConstants.Flywheel.SETPOINT_TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    masterDisconnectedAlert.set(!inputs.masterConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);

    Logger.processInputs("Shooter/Flywheel", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            io.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        kS,
        kV,
        kA,
        kP,
        kD);
  }

  @Override
  public SysIdRoutine getRoutine() {
    return sysIdRoutine;
  }
}
