package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

public class PivotIOTalonFXWithCANcoder extends PivotIOTalonFX {
  private final CANcoder cancoder;
  private final StatusSignal<Angle> absolutePositionSignal;

  private final double cancoderToMechanismRatio;

  @SuppressWarnings("removal")
  public PivotIOTalonFXWithCANcoder(
      int motorCanId,
      int cancoderCanId,
      String canBus,
      PivotSpecifications pivotSpecs,
      CANcoderSpecifications CANcoderSpecs) {
    super(motorCanId, canBus, pivotSpecs);

    cancoder = new CANcoder(cancoderCanId, canBus);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection =
        CANcoderSpecs.clockwisePositive()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> cancoder.getConfigurator().apply(config));

    absolutePositionSignal = cancoder.getAbsolutePosition();

    cancoderToMechanismRatio = CANcoderSpecs.gearRatio();

    seedMotorFromCancoder(5);
  }

  private void seedMotorFromCancoder(int attempts) {
    boolean refreshSignalSuccessful = false;

    for (int i = 0; i < attempts; i++) {
      if (BaseStatusSignal.refreshAll(absolutePositionSignal).isOK()) {
        refreshSignalSuccessful = true;
        break;
      }
    }

    if (!refreshSignalSuccessful) return;

    double cancoderRotations = absolutePositionSignal.getValue().in(Rotations);
    double mechanismRotations = cancoderRotations * cancoderToMechanismRatio;

    Logger.recordOutput("Expected Mechanism Rotations", mechanismRotations);

    tryUntilOk(5, () -> motor.setPosition(mechanismRotations));
  }
}
