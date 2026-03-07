package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Extension extends SubsystemBase {
  private final String name;

  private final ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
  public final ExtensionIO io;

  public Extension(String name, ExtensionIO io) {
    this.name = name;
    this.io = io;
  }

  public Command setPosition(Distance position) {
    return runOnce(() -> io.setPosition(position));
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name + "/Motor", inputs);
  }
}
