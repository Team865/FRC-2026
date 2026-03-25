package frc.robot.subsystems.rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  public final String name;
  public final RollersIO io;
  protected final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  protected final Alert motorDisconnectedAlert;

  // private final SysIdRoutine sysIdRoutine;

  public Rollers(String name, RollersIO io) {
    this.name = name;
    this.io = io;
    this.motorDisconnectedAlert = new Alert(name + " motor disconnected.", AlertType.kError);

    // sysIdRoutine =
    //     new SysIdBuilder(this, io::setVolts)
    //         .withDynamicStepVoltage(9.0)
    //         .withQuasistaticRampRate(0.5)
    //         .build();
  }

  public Command setVolts(double volts) {
    return this.runOnce(() -> io.setVolts(volts));
  }

  public Command setAngularVelocity(AngularVelocity velocity) {
    return this.runOnce(() -> io.setAngularVelocity(velocity));
  }

  public Command runVolts(double volts) {
    return this.runEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runVolts(DoubleSupplier voltageSupplier) {
    return this.runEnd(() -> io.setVolts(voltageSupplier.getAsDouble()), () -> io.setVolts(0));
  }

  public Command runAngularVelocity(AngularVelocity velocity) {
    return this.runEnd(() -> io.setAngularVelocity(velocity), () -> io.stop());
  }

  public Command runLinearVelocity(LinearVelocity velocity) {
    return this.runEnd(() -> io.setLinearVelocity(velocity), () -> io.stop());
  }

  public Command runLinearVelocity(Supplier<LinearVelocity> velocitySupplier) {
    return this.runEnd(() -> io.setLinearVelocity(velocitySupplier.get()), () -> io.stop());
  }

  public Command stop() {
    return this.runOnce(io::stop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    motorDisconnectedAlert.set(!inputs.connected);
    Logger.processInputs(name + "/Motor", inputs);
  }

  public Angle getPosition() {
    return inputs.position;
  }

  public AngularVelocity getAngularVelocity() {
    return inputs.angularVelocity;
  }

  // @Override
  // public SysIdRoutine getRoutine() {
  //   return sysIdRoutine;
  // }
}
