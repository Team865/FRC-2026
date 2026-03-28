package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface AbsoluteEncoderIO {
  @AutoLog
  public static class AbsoluteEncoderInputs {
    public boolean connected = false;
    public Angle position = Rotations.zero();
    public Angle positionPreGearing = Rotations.zero();
  }

  public default void updateInputs(AbsoluteEncoderInputs inputs) {}
}
