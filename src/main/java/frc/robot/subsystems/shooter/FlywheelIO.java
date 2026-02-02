package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;

    public double masterAppliedVoltage = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterStatorCurrentAmps = 0.0;
    public boolean masterConnected = false;

    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerStatorCurrentAmps = 0.0;
    public boolean followerConnected = false;
  }

  public default void updateInputs(FlywheelIOInputsAutoLogged inputs) {}

  public default void setVolts(double volts) {}

  public default void setVelocity(double velocityRadPerSec) {}

  public default void setControlConstants(double kS, double kV, double kP, double kD) {}

  public default void stop() {}
}
