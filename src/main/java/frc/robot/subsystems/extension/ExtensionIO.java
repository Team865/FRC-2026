package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ExtensionIO {
  @AutoLog
  public static class ExtensionIOInputs {
    public boolean connected = false;
    public Distance targetPosition = Meters.zero();
    public Distance position = Meters.zero();
    public LinearVelocity velocity = MetersPerSecond.zero();
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public default void updateInputs(ExtensionIOInputsAutoLogged inputs) {}

  public default void setVolts(double volts) {}

  public default void setPosition(Distance position) {}

  public default void stop() {}

  public default void setControlConstants(double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(double maxVelocity, double maxAcceleration) {}
}
