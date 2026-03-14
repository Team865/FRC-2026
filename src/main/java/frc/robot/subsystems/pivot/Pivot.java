package frc.robot.subsystems.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  public final String name;

  public final PivotIO io;
  protected final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  protected final Alert motorDisconnectedAlert;

  public Pivot(String name, PivotIO io) {
    this.name = name;
    this.io = io;
    this.motorDisconnectedAlert = new Alert(name + " motor disconnected", AlertType.kError);
  }

  public Command setVoltage(double voltage) {
    return this.runOnce(() -> io.setVolts(voltage));
  }

  public Command runVoltage(double voltage) {
    return this.runEnd(() -> io.setVolts(voltage), io::stop);
  }

  public Command runVoltage(DoubleSupplier voltageSupplier) {
    return this.runEnd(() -> io.setVolts(voltageSupplier.getAsDouble()), io::stop);
  }

  public Command setTargetAngle(Angle target) {
    return this.runOnce(() -> io.setPosition(target));
  }

  public Command setTargetAngle(Supplier<Angle> targetSupplier) {
    return this.runOnce(() -> io.setPosition(targetSupplier.get()));
  }

  public Command runTargetAngle(Supplier<Angle> targetSupplier) {
    return this.runEnd(() -> io.setPosition(targetSupplier.get()), io::stop);
  }

  public Command stop() {
    return this.runOnce(io::stop);
  }

  public Angle getOrientation() {
    return inputs.position;
  }

  public Angle getTargetOrientation() {
    return inputs.targetPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    motorDisconnectedAlert.set(!inputs.connected);
    Logger.processInputs(name + "/Motor", inputs);
  }
}
