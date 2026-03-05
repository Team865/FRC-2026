package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.Quarter;

public class LEDs extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private double wavePhase = 0; // offsets for animation

  public LEDs() {
    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(numLeds);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  // set a specific led to a color
  public void setLED(int index, Color color) {
    buffer.setRGB(
        index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    led.setData(buffer);
  }

  // Helper to set every LED to a color
  public void setAll(Color color) {
    int r = (int) (color.red * 255);
    int g = (int) (color.green * 255);
    int b = (int) (color.blue * 255);

    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }

  // Set a section to a color
  public void setSection(int startIndex, int endIndex, Color color) {
    startIndex = Math.max(0, startIndex);
    endIndex = Math.min(buffer.getLength(), endIndex);
    int r = (int) (color.red * 255);
    int g = (int) (color.green * 255);
    int b = (int) (color.blue * 255);

    for (int i = startIndex; i < endIndex; i++) {
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }

  // Helper to set a named quarter using constants
  public void setSideColor(Quarter quarter, Color color) {
    int quarterLength = buffer.getLength() / 4;
    int start = quarterLength * quarter.getIndex();
    int end = (quarter == Quarter.LeftSide) ? buffer.getLength() : start + quarterLength;
    setSection(start, end, color);
  }

  public Command setSideColorCommand(Quarter quarter, Color color) {
    return runOnce(() -> setSideColor(quarter, color));
  }

  public void allianceColorWave() {
    int length = buffer.getLength();
    Color allianceColor = ALLIANCE_COLOR;
    for (int i = 0; i < length; i++) {
      double wave = (Math.sin((i * 0.3) + wavePhase) + 1) / 2.0;

      int r = (int) (allianceColor.red * 255 * wave);
      int g = (int) (allianceColor.green * 255 * wave);
      int b = (int) (allianceColor.blue * 255 * wave);

      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
    wavePhase += 0.1;
    if (wavePhase > 2 * Math.PI) wavePhase -= 2 * Math.PI;
  }

  public Command allianceColorWaveCommand() {
    return run(() -> allianceColorWave());
  }
}
