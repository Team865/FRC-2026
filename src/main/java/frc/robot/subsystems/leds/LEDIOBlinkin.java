package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIOBlinkin implements LEDIO {

  private final Spark blinkin;
  private double appliedPattern = LEDConstants.OFF;

  public LEDIOBlinkin() {
    blinkin = new Spark(LEDConstants.pwmPort);
  }

  @Override
  public void setPattern(double pattern) {
    appliedPattern = pattern;
    blinkin.set(pattern);
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.appliedPattern = appliedPattern;
  }
}
