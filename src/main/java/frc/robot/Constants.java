package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  private static final boolean enableLockoutZone = false;

  // Allow tuning in SIM or REAl, disallow in a competition match or REPLAY
  public static boolean tuningMode() {
    switch (currentMode) {
      case SIM:
        return true;

      case REAL:
        return !DriverStation.isFMSAttached();

      case REPLAY:
      default:
        return false;
    }
  }

  // Prevent the robot from moving to certain zones in the shop
  public static boolean shouldUseLockoutZones() {
    return switch (currentMode) {
      case REAL -> !DriverStation.isFMSAttached() && enableLockoutZone;
      case SIM -> enableLockoutZone; // Remove later
      default -> false;
    };
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public record ControlSystemContext(
      double kV,
      double kA,
      double kS,
      double kG,
      double kP,
      double kD,
      Optional<Double> maxVelocity,
      Optional<Double> maxAcceleration) {
    public ControlSystemContext(double kV, double kA, double kS, double kG, double kP, double kD) {
      this(kV, kA, kS, kG, kP, kD, Optional.empty(), Optional.empty());
    }
  }

  public static class ControlSystemConstants {
    public static final ControlSystemContext EMPTY_CONTEXT =
        new ControlSystemContext(0, 0, 0, 0, 0, 0, Optional.of(0.0), Optional.of(0.0));
    public final ControlSystemContext REAL_CONTEXT;
    public final ControlSystemContext SIM_CONTEXT;

    public final double kV;
    public final double kA;
    public final double kS;
    public final double kG;
    public final double kP;
    public final double kD;
    public final Optional<Double> maxVelocity;
    public final Optional<Double> maxAcceleration;

    public ControlSystemConstants(
        ControlSystemContext realContext, ControlSystemContext simContext) {
      REAL_CONTEXT = realContext;
      SIM_CONTEXT = simContext;

      ControlSystemContext context = (currentMode.equals(Mode.REAL)) ? realContext : simContext;

      kV = context.kV();
      kA = context.kA();
      kS = context.kS();
      kG = context.kG();
      kP = context.kP();
      kD = context.kD();

      maxVelocity = context.maxVelocity();
      maxAcceleration = context.maxAcceleration();
    }
  }
}
