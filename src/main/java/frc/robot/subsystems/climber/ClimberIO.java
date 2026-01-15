package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = false;
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double positionMeters = 0.0;
  }

  public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double positionMeters) {}

  public default void setControlGains(
      double kG, double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(
      double maxVelocity, double maxAcceleration, double maxJerk) {}

  public default void stop() {}
}
