package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
  private LEDConstants() {}

  public static final int pwmPort = 1; // PWM port the leds are running on
  public static final int totalNumLeds = 189;

  public static final int rightLeds = 62;
  public static final int leftLeds = 62;
  public static final int turretLeds = 65;

  public enum PresetColor {
    RED(new Color(1.0, 0.0, 0.0)),
    GREEN(new Color(0.0, 1.0, 0.0)),
    BLUE(new Color(0.0, 0.0, 1.0)),
    YELLOW(new Color(1.0, 1.0, 0.0)),
    MAGENTA(new Color(1.0, 0.0, 1.0)),
    CYAN(new Color(0.0, 1.0, 1.0)),
    WHITE(new Color(1.0, 1.0, 1.0)),
    OFF(new Color(0.0, 0.0, 0.0)),
    ORANGE(new Color(1.0, 0.2, 0.0)),
    PURPLE(new Color(0.5, 0.0, 0.5)),
    PINK(new Color(1.0, 0.4, 0.7)),
    LIGHT_BLUE(new Color(0.0, 0.7, 1.0)),
    LIME(new Color(0.5, 1.0, 0.0)),
    TEAL(new Color(0.0, 0.5, 0.5)),
    GREY(new Color(0.5, 0.5, 0.5)),

    IDLE(new Color(120.0 / 255.0, 0.0, 200.0 / 255.0));

    public final Color color;

    PresetColor(Color color) {
      this.color = color;
    }
  }

  public enum Section {
    Right(0, rightLeds),
    Left(rightLeds, rightLeds + leftLeds),
    Turret(rightLeds + leftLeds, rightLeds + leftLeds + turretLeds),
    Overflow(rightLeds + leftLeds + turretLeds, totalNumLeds);

    private final int start;
    private final int end;

    Section(int start, int end) {
      this.start = start;
      this.end = end;
    }

    public int getStart() {
      return start;
    }

    public int getEnd() {
      return end;
    }

    public int getLength() {
      return end - start;
    }
  }
}
