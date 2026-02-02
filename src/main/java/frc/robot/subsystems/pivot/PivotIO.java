package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  // velocity and position are measured in mechanism units not motor units
  public static class PivotIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double torqueCurrent = 0.0;
    public boolean connected = false;
  }

  public default void setVolts(double volts) {}

  public default void setPosition(double angleRads) {}

  public default void setPositionWithExtraOmega(double angleRads, double omegaRadPerSec) {}

  public default void stop() {}

  public default void updateInputs(PivotIOInputsAutoLogged inputs) {}

  public default void setControlConstants(double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(double maxVelocity, double maxAcceleration) {}
}
