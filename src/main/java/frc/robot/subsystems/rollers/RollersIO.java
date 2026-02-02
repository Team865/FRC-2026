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

  default void updateInputs(RollersIOInputsAutoLogged inputs) {}

  default void setVolts(double volts) {}

  default void setVelocity(double velocityRadPerSec) {}

  default void setControlConstants(double kS, double kV, double kP, double kD) {}

  default double getVelocityRadPerSec() {
    return 0.0;
  }
}
