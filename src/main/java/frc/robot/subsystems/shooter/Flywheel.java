package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
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
      new LoggedTunableNumber("Shooter/Flywheel/kV", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Flywheel/kS", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Flywheel/kP", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Flywheel/kD", ShooterConstants.Flywheel.SYSTEM_CONSTANTS.kD);

  public Flywheel(FlywheelIO io) {
    this.io = io;

    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  public Command setVolts(double volts) {
    return this.runOnce(() -> io.setVolts(volts));
  }

  public Command setVelocity(AngularVelocity velocity) {
    return this.runOnce(
        () -> {
          targetVelocityRadsPerSec = velocity.in(RadiansPerSecond);
          io.setVelocity(velocity.in(RadiansPerSecond));
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
          targetVelocityRadsPerSec = velocity.in(RadiansPerSecond);
          io.setVelocity(targetVelocityRadsPerSec);
        },
        () -> {
          targetVelocityRadsPerSec = 0.0;
          io.stop();
        });
  }

  public Command runVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return this.runEnd(
        () -> {
          targetVelocityRadsPerSec = velocitySupplier.get().in(RadiansPerSecond);
          io.setVelocity(targetVelocityRadsPerSec);
        },
        () -> {
          targetVelocityRadsPerSec = 0.0;
          io.stop();
        });
  }

  public Command stop() {
    return this.runOnce(io::stop);
  }

  public Trigger atTargetVelocity() {
    return new Trigger(
        () ->
            MathUtil.isNear(
                targetVelocityRadsPerSec,
                inputs.velocityRadsPerSec,
                ShooterConstants.Flywheel.SETPOINT_TOLERANCE_RADS));
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
