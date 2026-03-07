package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  class RollersIOInputs {
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public boolean connected = false;
  }

  public default void updateInputs(RollersIOInputsAutoLogged inputs) {}

  public default void setVolts(double volts) {}

  public default void stop() {}

  public default void setAngularVelocity(AngularVelocity velocity) {}

  public default void setLinearVelocity(LinearVelocity velocity) {}

  public default void setControlConstants(double kS, double kV, double kA, double kP, double kD) {}
}
