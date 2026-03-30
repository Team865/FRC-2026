package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public Angle position = Rotations.zero();
    public AngularVelocity masterVelocity = RotationsPerSecond.zero();
    public AngularVelocity followerVelocity = RotationsPerSecond.zero();

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

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setControlConstants(double kS, double kV, double kA, double kP, double kD) {}

  public default void stop() {}
}
