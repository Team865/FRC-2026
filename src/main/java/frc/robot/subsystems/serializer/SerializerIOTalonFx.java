package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class SerializerIOTalonFx implements SerializerIO {
  private final TalonFX talon;
  private final TalonFXConfiguration config;
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(50.0);
  private final Debouncer connectedDebouncer = new Debouncer(0.5);
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public SerializerIOTalonFx(int talonCANId, String canbus) {
    talon = new TalonFX(talonCANId, canbus);
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kS = SerializerConstants.kS;
    config.Slot0.kV = SerializerConstants.kV;
    config.Slot0.kP = SerializerConstants.kP;
    config.Slot0.kD = SerializerConstants.kD;
    config.Slot0.kI = 0.0;
    config.Feedback.SensorToMechanismRatio =
        SerializerConstants.GEAR_RATIO; // Adjust for gearing constant
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
  }

  @Override
  public void updateInputs(SerializerIOInputsAutoLogged inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position, velocity, appliedVoltage, supplyCurrent, torqueCurrent)
                .isOK());

    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kP, double kD) {

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    talon.setControl(velocityVoltage.withVelocity(velocityRadPerSec));
  }
}
