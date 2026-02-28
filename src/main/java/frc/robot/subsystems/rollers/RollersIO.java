package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  class RollersIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public boolean connected = false;
  }

  public default void updateInputs(RollersIOInputsAutoLogged inputs) {}

  public default void setVolts(double volts) {}

  public default void stop() {}

  public default void setAngularVelocity(double velocityRadPerSec) {}

  public default void setLinearVelocity(double velocityMetersPerSec) {}

  public default void setControlConstants(double kV, double kA, double kS, double kP, double kD) {}

  public default double getVelocityRadPerSec() {
    return 0.0;
  }
}
