package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class LEDs extends SubsystemBase {

  private final Spark blinkin = new Spark(LEDConstants.pwmPort);

  private double currentPattern = LEDConstants.OFF;

  public LEDs() {
    blinkin.set(currentPattern);
  }

  public void set(double pattern) {
    currentPattern = pattern;
    blinkin.set(pattern);
  }

  @AutoLogOutput(key = "LEDs/AppliedPattern")
  public double getCurrentPattern() {
    return currentPattern;
  }
}
