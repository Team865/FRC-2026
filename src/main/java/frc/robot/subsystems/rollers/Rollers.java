package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  public final String name;
  protected final RollersIO io;
  protected final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(String name, RollersIO io) {
    this.name = name;
    this.io = io;
  }

  public Command runVolts(double volts) {
    return this.runEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runVelocity(double velocityRadPerSec) {
    return this.runEnd(() -> io.setVelocity(velocityRadPerSec), () -> io.setVelocity(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public RollersIO getIO() {
    return io;
  }
}
