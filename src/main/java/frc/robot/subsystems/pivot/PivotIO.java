package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  // velocity and position are measured in mechanism units not motor units
  public static class PivotIOInputs {
    public Angle targetPosition = Rotations.zero();
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public boolean connected = false;
  }

  public default void setVolts(double volts) {}

  public default void setPosition(Angle angleRads) {}

  public default void setPositionWithExtraOmega(Angle angleRads, AngularVelocity omega) {}

  public default void stop() {}

  public default void updateInputs(PivotIOInputsAutoLogged inputs) {}

  public default void setControlConstants(double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(double maxVelocity, double maxAcceleration) {}

  public default boolean seedPosition(Angle position) {
    return true;
  }
}
