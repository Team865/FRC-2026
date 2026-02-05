package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public boolean connected = false;
  }

  public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPosition(double positionMeters) {}

  public default void setControlGains(
      double kG, double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(double maxVelocity, double maxAcceleration) {}

  public default void stop() {}
}
