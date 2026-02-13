package frc.robot.subsystems.rollers;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  public final String name;
  protected final RollersIO io;
  protected final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(String name, RollersIO io) {
    this.name = name;
    this.io = io;
  }

  public Command setVolts(double volts) {
    return this.runOnce(() -> io.setVolts(volts));
  }

  public Command setAngularVelocity(AngularVelocity velocity) {
    return this.runOnce(() -> io.setAngularVelocity(velocity.in(RadiansPerSecond)));
  }

  public Command runVolts(double volts) {
    return this.runEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runAngularVelocity(AngularVelocity velocity) {
    return this.runEnd(() -> io.setAngularVelocity(velocity.in(RadiansPerSecond)), () -> io.stop());
  }

  public Command runLinearVelocity(LinearVelocity velocity) {
    return this.runEnd(() -> io.setLinearVelocity(velocity.in(MetersPerSecond)), () -> io.stop());
  }

  public Command runLinearVelocity(Supplier<LinearVelocity> velocitySupplier) {
    return this.runEnd(
        () -> io.setLinearVelocity(velocitySupplier.get().in(MetersPerSecond)), () -> io.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public RollersIO getIO() {
    return io;
  }

  public double getPositionRads() {
    return inputs.positionRads;
  }
}
