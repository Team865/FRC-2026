package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.AllianceFlipUtil;

public final class LEDConstants {
  private LEDConstants() {}

  public static final int pwmPort = 1; // Pwm port the leds are running on
  public static final int numLeds = 60; // Number of leds, this is 1 meter of lights by default

  public static final Color RED = new Color(1.0, 0.0, 0.0);
  public static final Color GREEN = new Color(0.0, 1.0, 0.0);
  public static final Color BLUE = new Color(0.0, 0.0, 1.0);
  public static final Color YELLOW = new Color(1.0, 1.0, 0.0);
  public static final Color MAGENTA = new Color(1.0, 0.0, 1.0);
  public static final Color CYAN = new Color(0.0, 1.0, 1.0);
  public static final Color WHITE = new Color(1.0, 1.0, 1.0);
  public static final Color OFF = new Color(0.0, 0.0, 0.0);
  public static final Color ORANGE = new Color(1.0, 0.5, 0.0);
  public static final Color PURPLE = new Color(0.5, 0.0, 0.5);
  public static final Color PINK = new Color(1.0, 0.4, 0.7);
  public static final Color LIGHT_BLUE = new Color(0.0, 0.7, 1.0);
  public static final Color LIME = new Color(0.5, 1.0, 0.0);
  public static final Color TEAL = new Color(0.0, 0.5, 0.5);
  public static final Color GREY = new Color(0.5, 0.5, 0.5);
  public static final Color ALLIANCE_COLOR = AllianceFlipUtil.shouldFlip() ? RED : BLUE;

  public enum Quarter {
    TurretSide(0),
    RightSide(1),
    IntakeSide(2),
    LeftSide(3);

    private final int index;

    Quarter(int index) {
      this.index = index;
    }

    // Getter
    public int getIndex() {
      return index;
    }
  }
}
