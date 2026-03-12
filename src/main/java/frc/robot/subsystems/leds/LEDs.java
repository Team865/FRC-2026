package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.leds.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.PresetColor;
import frc.robot.subsystems.leds.LEDConstants.Section;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

public class LEDs extends SubsystemBase {

  public final AddressableLED led;
  public final AddressableLEDBuffer buffer;
  private double wavePhase = 0; // offsets for animation

  private static final LoggedTunableNumber testRed = new LoggedTunableNumber("TestLed/red", 1.0);
  private static final LoggedTunableNumber testGreen =
      new LoggedTunableNumber("TestLed/green", 1.0);
  private static final LoggedTunableNumber testBlue = new LoggedTunableNumber("TestLed/blue", 1.0);

  private final LEDPattern scrollingRainbow =
      LEDPattern.rainbow(255, 128)
          .scrollAtAbsoluteSpeed(MetersPerSecond.of(5), Meters.of(1.0 / 60));

  public LEDs() {
    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(totalNumLeds);

    led.setLength(totalNumLeds);
    led.setData(buffer);
    led.start();
  }

  private void setLED(int ledIndex, int red, int green, int blue) {
    // Red and Green are flipped for some reason
    buffer.setRGB(ledIndex, green, red, blue);
  }

  // set a specific led to a color
  public void setLED(int index, Color color) {
    setLED(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  // Helper to set every LED to a color
  public void setAll(int r, int g, int b) {
    for (int i = 0; i < totalNumLeds; i++) {
      setLED(i, r, g, b);
    }

    // Update LED
    led.setData(buffer);
  }

  public void setAll(Color color) {
    int r = (int) (color.red * 255);
    int g = (int) (color.green * 255);
    int b = (int) (color.blue * 255);

    setAll(r, g, b);
  }

  // Set a section to a color
  public void setRange(int startIndex, int endIndex, Color color) {
    startIndex = Math.max(0, startIndex);
    endIndex = Math.min(totalNumLeds, endIndex);
    int r = (int) (color.red * 255);
    int g = (int) (color.green * 255);
    int b = (int) (color.blue * 255);

    for (int i = startIndex; i < endIndex; i++) {
      setLED(i, r, g, b);
    }

    led.setData(buffer);
  }

  // Helper to set a named section using constants
  public void setSection(Section section, Color color) {
    int quarterLength = numTableLeds / 4;
    int start = quarterLength * section.getIndex();
    int end = (section == Section.BellyPan) ? numBellyPanLeds : start + quarterLength;
    setRange(start, end, color);

    led.setData(buffer);
  }

  public Command setSideColorCommand(Section section, Color color) {
    return runOnce(() -> setSection(section, color));
  }

  private double sampleSawtooth(double x, double amplitude, double period) {
    // Limit x to the period
    x = x >= 0 ? x % period : period + (x % period);

    double halfPeriod = period / 2;

    if (x <= halfPeriod) {
      return 2 * (amplitude / period) * x - amplitude / 2;
    } else {
      return -2 * (amplitude / period) * x + 3 * (amplitude / 2);
    }
  }

  public void updateWave(
      Color color,
      double minBrightness,
      double maxBrightness,
      double numPeriods,
      double waveRate,
      int waveDegree) {

    double period = 1.0 / numPeriods;

    double range = maxBrightness - minBrightness;

    for (int i = 0; i < totalNumLeds; i++) {
      double x = (double) i / totalNumLeds;

      double wave = sampleSawtooth(x - wavePhase, 1.0, period) + 0.5;

      // Bias lower values
      wave = Math.pow(wave, waveDegree);

      wave = minBrightness + wave * range; // Fit to min and max

      int r = (int) (color.red * 255 * wave);
      int g = (int) (color.green * 255 * wave);
      int b = (int) (color.blue * 255 * wave);

      setLED(i, r, g, b);
    }

    led.setData(buffer);

    wavePhase += waveRate * 0.020;
    if (wavePhase > period) wavePhase -= period;
  }

  public void updateWave(
      Color color, double minBrightness, double maxBrightness, double numPeriods, double waveRate) {
    updateWave(color, minBrightness, maxBrightness, numPeriods, waveRate, 1);
  }

  public void updateRainbowWave() {
    scrollingRainbow.applyTo(buffer);
    led.setData(buffer);
  }

  public void updateAllianceColorWave() {
    if (AllianceFlipUtil.shouldFlip()) {
      updateWave(PresetColor.RED.color, 0.02, 0.8, 4, 0.3, 2);
    } else {
      updateWave(PresetColor.BLUE.color, 0.025, 0.8, 4, 0.3, 3);
    }
  }

  public Command allianceColorWaveCommand() {
    return run(() -> updateAllianceColorWave());
  }

  public Command shootingWaveCommand() {
    return run(() -> updateWave(PresetColor.IDLE.color, 0.02, 0.6, 4, 0.9, 2));
  }

  public Command idleWaveCommand() {
    return run(() -> updateWave(PresetColor.IDLE.color, 0.02, 0.6, 4, 0.3, 2));
  }

  public Command testColour() {
    return runEnd(
        () -> updateWave(PresetColor.IDLE.color, 0.02, 0.6, 4, 0.3, 2), () -> setAll(Color.kBlack));
  }
}
