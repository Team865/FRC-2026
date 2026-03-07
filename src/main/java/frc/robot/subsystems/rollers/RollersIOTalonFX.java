package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX implements RollersIO {

  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final RollersSpecifications specs;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(50.0);
  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(50.0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;

  @SuppressWarnings("removal")
  public RollersIOTalonFX(int canId, String canBus, RollersSpecifications specs) {
    this.specs = specs;
    talon = new TalonFX(canId, canBus);

    config.MotorOutput.Inverted =
        specs.clockwisePositive()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = specs.supplyCurrentLimit();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = specs.statorCurrentLimit();

    config.Feedback.SensorToMechanismRatio = specs.gearRatio();

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(0.0));

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
  public void updateInputs(RollersIOInputsAutoLogged inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position, velocity, appliedVoltage, supplyCurrent, torqueCurrent)
                .isOK());

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    talon.setControl(velocityVoltage.withVelocity(velocity));
  }

  @Override
  public void setLinearVelocity(LinearVelocity velocity) {
    talon.setControl(
        velocityVoltage.withVelocity(
            RadiansPerSecond.of(velocity.in(MetersPerSecond) / specs.rollerRadiusMeters())));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
