package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Extension extends FullSubsystem {
  private final String name;

  public final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
  public final ExtensionIO io;

  protected Distance atSetpointTolerance = Inches.of(1.0);

  // private final SysIdRoutine sysIdRoutine;

  public Extension(String name, ExtensionIO io) {
    this.name = name;
    this.io = io;

    // sysIdRoutine =
    //     new SysIdBuilder(this, io::setVolts)
    //         .withDynamicStepVoltage(2)
    //         .withQuasistaticRampRate(0.2)
    //         .build();
  }

  public Command setPosition(Distance position) {
    return runOnce(() -> io.setPosition(position));
  }

  public Command setPosition(Supplier<Distance> positionSupplier) {
    return runOnce(() -> io.setPosition(positionSupplier.get()));
  }

  public Trigger atSetpoint() {
    return new Trigger(() -> inputs.position.isNear(inputs.targetPosition, Inches.of(0.5)));
  }

  public Command runPosition(Distance position) {
    return runEnd(() -> io.setPosition(position), io::stop);
  }

  public Command setVolts(double volts) {
    return runOnce(() -> io.setVolts(volts));
  }

  public Command runVolts(double volts) {
    return runEnd(() -> io.setVolts(volts), io::stop);
  }

  public Command runVolts(DoubleSupplier voltageSupplier) {
    return runEnd(() -> io.setVolts(voltageSupplier.getAsDouble()), io::stop);
  }

  public Command stop() {
    return runOnce(io::stop);
  }

  public Distance getPosition() {
    return inputs.position;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name + "/Motor", inputs);
  }

  public boolean isAtSetpoint(Distance setpoint) {
    return inputs.position.isNear(inputs.targetPosition, atSetpointTolerance);
  }

  // @Override
  // public SysIdRoutine getRoutine() {
  //   return sysIdRoutine;
  // }
}
