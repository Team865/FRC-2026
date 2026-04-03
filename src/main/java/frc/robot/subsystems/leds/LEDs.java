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

  private double wavePhase = 0;

  private static final LoggedTunableNumber testRed = new LoggedTunableNumber("TestLed/red", 1.0);
  private static final LoggedTunableNumber testGreen =
      new LoggedTunableNumber("TestLed/green", 1.0);
  private static final LoggedTunableNumber testBlue = new LoggedTunableNumber("TestLed/blue", 1.0);

  private final LEDPattern scrollingRainbow =
      LEDPattern.rainbow(255, 128)
          .scrollAtAbsoluteSpeed(MetersPerSecond.of(4), Meters.of(2.0 / 60));

  public LEDs() {
    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(totalNumLeds);

    led.setLength(totalNumLeds);
    led.setData(buffer);
    led.start();
  }

  private void setLED(int index, int r, int g, int b) {
    // fix rbg swap
    buffer.setRGB(index, g, r, b);
  }

  public void setLED(int index, Color color) {
    setLED(index, to255(color.red), to255(color.green), to255(color.blue));
  }

  private int to255(double value) {
    return (int) (value * 255);
  }

  public void setAll(Color color) {
    for (int i = 0; i < totalNumLeds; i++) {
      setLED(i, color);
    }
    led.setData(buffer);
  }

  public void setAll(PresetColor color) {
    setAll(color.color);
  }

  public void setRange(int start, int end, Color color) {
    start = Math.max(0, start);
    end = Math.min(totalNumLeds, end);

    for (int i = start; i < end; i++) {
      setLED(i, color);
    }

    led.setData(buffer);
  }

  public void setSection(Section section, Color color) {
    setRange(section.getStart(), section.getEnd(), color);
  }

  public void setSection(Section section, PresetColor color) {
    setSection(section, color.color);
  }

  public Command setSectionCommand(Section section, PresetColor color) {
    return runOnce(() -> setSection(section, color));
  }

  // wave math

  private double sampleSawtooth(double x, double amplitude, double period) {
    x = x >= 0 ? x % period : period + (x % period);

    double half = period / 2;

    if (x <= half) {
      return 2 * (amplitude / period) * x - amplitude / 2;
    } else {
      return -2 * (amplitude / period) * x + 3 * (amplitude / 2);
    }
  }

  // full strip wave

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
      wave = Math.pow(wave, waveDegree);
      wave = minBrightness + wave * range;

      setLED(
          i,
          (int) (color.red * 255 * wave),
          (int) (color.green * 255 * wave),
          (int) (color.blue * 255 * wave));
    }

    led.setData(buffer);

    wavePhase += waveRate * 0.020;
    if (wavePhase > period) wavePhase -= period;
  }

  public void updateWave(
      Color color, double minBrightness, double maxBrightness, double numPeriods, double waveRate) {
    updateWave(color, minBrightness, maxBrightness, numPeriods, waveRate, 1);
  }

  // section only wave

  public void updateWave(
      Section section,
      Color color,
      double minBrightness,
      double maxBrightness,
      double numPeriods,
      double waveRate,
      int waveDegree) {

    double period = 1.0 / numPeriods;
    double range = maxBrightness - minBrightness;

    int start = section.getStart();
    int end = section.getEnd();
    int length = section.getLength();

    for (int i = start; i < end; i++) {
      double x = (double) (i - start) / length;

      double wave = sampleSawtooth(x - wavePhase, 1.0, period) + 0.5;
      wave = Math.pow(wave, waveDegree);
      wave = minBrightness + wave * range;

      setLED(
          i,
          (int) (color.red * 255 * wave),
          (int) (color.green * 255 * wave),
          (int) (color.blue * 255 * wave));
    }

    led.setData(buffer);

    wavePhase += waveRate * 0.020;
    if (wavePhase > period) wavePhase -= period;
  }

  public void updateWave(
      Section section,
      PresetColor color,
      double minBrightness,
      double maxBrightness,
      double numPeriods,
      double waveRate,
      int waveDegree) {

    updateWave(
        section, color.color, minBrightness, maxBrightness, numPeriods, waveRate, waveDegree);
  }

  public void updateRainbowWave() {
    scrollingRainbow.applyTo(buffer);
    led.setData(buffer);
  }

  public void updateAllianceColorWave() {
    if (AllianceFlipUtil.shouldFlip()) {
      updateWave(PresetColor.RED.color, 0.02, 0.8, 4, 0.3, 2);
    } else {
      updateWave(PresetColor.BLUE.color, 0.02, 0.8, 4, 0.3, 2);
    }
  }

  public Command allianceColorWaveCommand() {
    return run(this::updateAllianceColorWave);
  }

  public Command shootingActiveWaveCommand() {
    return run(() -> updateWave(Section.Turret, PresetColor.YELLOW, 0.02, 0.8, 5, 0.99, 2));
  }

  public Command autoShootingActiveWaveCommand() {
    return run(() -> updateWave(PresetColor.GREEN.color, 0.02, 0.6, 4, 0.99, 2));
  }

  public Command shootingIdleWaveCommand() {
    return run(() -> updateWave(Section.Turret, PresetColor.IDLE, 0.02, 0.8, 5, 0.99, 2));
  }

  public Command idleWaveCommand() {
    return run(() -> updateWave(PresetColor.IDLE.color, 0.02, 0.6, 4, 0.3, 2));
  }

  public Command testColour() {
    return runEnd(
        () -> setAll(new Color(testRed.get(), testGreen.get(), testBlue.get())),
        () -> setAll(Color.kBlack));
  }
}
