package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
  private LEDConstants() {}

  public static final int pwmPort = 1; // Pwm port the leds are running on
  public static final int totalNumLeds =
      189; // Number of leds, this is 1 meter of lights by default
  public static final int rightLeds = 62;
  public static final int leftLeds = 62;
  public static final int igusLeds = 65;

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

    IDLE(new Color(120.0 / 255, 0, 200.0 / 255));

    public final Color color;

    private PresetColor(Color color) {
      this.color = color;
    }
  }

  public enum Section {
    Right(0),
    Left(1),
    Turret(2),
    Overflow(3);

    private final int index;

    Section(int index) {
      this.index = index;
    }

    // Getter
    public int getIndex() {
      return index;
    }
  }
}
