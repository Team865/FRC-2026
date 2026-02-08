package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {

  @AutoLog
  class LEDIOInputs {
    public double appliedPattern = LEDConstants.OFF;
  }

  default void updateInputs(LEDIOInputs inputs) {}

  default void setPattern(double pattern) {}
}
