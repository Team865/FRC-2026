package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class CANcoderIO implements AbsoluteEncoderIO {
  private final CANcoder cancoder;
  private final StatusSignal<Angle> absolutePositionSignal;

  private final double cancoderToMechanismRatio;
  private final Debouncer encoderDisconnectDebouncer = new Debouncer(0.5);

  @SuppressWarnings("removal")
  public CANcoderIO(int cancoderId, String canbus, CANcoderSpecifications CANcoderSpecifications) {
    cancoder = new CANcoder(cancoderId, canbus);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection =
        CANcoderSpecifications.clockwisePositive()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = CANcoderSpecifications.magnetOffsetRots();

    absolutePositionSignal = cancoder.getAbsolutePosition();
    Logger.recordOutput("Initial Offset Pre-Config", absolutePositionSignal.getValue());

    PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(config));

    Logger.recordOutput("Initial Offset", absolutePositionSignal.getValue());

    cancoderToMechanismRatio = CANcoderSpecifications.gearRatio();
  }

  @Override
  public void updateInputs(AbsoluteEncoderInputs inputs) {
    inputs.connected =
        encoderDisconnectDebouncer.calculate(absolutePositionSignal.refresh().getStatus().isOK());
    inputs.position = absolutePositionSignal.getValue();
  }
}
