package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {

  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  private double currentPattern = LEDConstants.OFF;

  public LEDs(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }

  public void set(double pattern) {
    currentPattern = pattern;
    io.setPattern(pattern);
  }

  public double getCurrentPattern() {
    return currentPattern;
  }
}
