package frc.robot.subsystems.leds;

public class LEDIOSim implements LEDIO {

  private double appliedPattern = LEDConstants.OFF;

  @Override
  public void setPattern(double pattern) {
    appliedPattern = pattern;
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.appliedPattern = appliedPattern;
  }
}
