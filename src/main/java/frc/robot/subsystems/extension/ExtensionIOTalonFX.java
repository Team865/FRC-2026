package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ExtensionIOTalonFX implements ExtensionIO {
  private final TalonFX talon;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final double drumRadiusMeters;

  private final VoltageOut voltageRequest =
      new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> positionAngleSignal;
  private final StatusSignal<AngularVelocity> angularVelocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> torqueCurrentSignal;

  private double extraGain = 0.0;
  private Angle extraGainTolerance = Rotations.zero();

  private Distance targetPosition = Meters.zero();

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  @SuppressWarnings("removal")
  public ExtensionIOTalonFX(int motorId, String canbus, ExtensionSpecifications specs) {
    talon = new TalonFX(motorId, canbus);

    drumRadiusMeters = specs.drumRadiusMeters();

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = specs.statorCurrentLimit();
    config.CurrentLimits.SupplyCurrentLimit = specs.supplyCurrentLimit();

    config.Feedback.SensorToMechanismRatio = specs.gearRatio();
    config.MotorOutput.Inverted =
        specs.clockwisePositive()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.5));

    positionAngleSignal = talon.getPosition();
    angularVelocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    supplyCurrentSignal = talon.getSupplyCurrent();
    statorCurrentSignal = talon.getStatorCurrent();
    torqueCurrentSignal = talon.getTorqueCurrent();

    PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(0.0));
    talon.optimizeBusUtilization();
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                Hertz.of(50.0),
                positionAngleSignal,
                angularVelocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                torqueCurrentSignal));
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Distance position) {
    targetPosition = position;

    talon.setControl(
        positionRequest.withPosition(Radians.of(position.in(Meters) / drumRadiusMeters)));
  }

  @Override
  public boolean seedPosition(Distance position) {
    Angle angularPosition = Radians.of(position.in(Meters) / drumRadiusMeters);

    return PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(angularPosition));
  }

  @Override
  public void stop() {
    talon.setControl(neutralRequest);
  }

  @Override
  public void updateInputs(ExtensionIOInputsAutoLogged inputs) {
    boolean refreshSucceeded =
        BaseStatusSignal.refreshAll(
                positionAngleSignal,
                angularVelocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                torqueCurrentSignal)
            .isOK();

    inputs.connected = connectedDebouncer.calculate(refreshSucceeded);

    if (!refreshSucceeded) return;

    inputs.position = Meters.of(positionAngleSignal.getValue().in(Radians) * drumRadiusMeters);
    inputs.targetPosition = targetPosition;
    inputs.velocity =
        MetersPerSecond.of(
            angularVelocitySignal.getValue().in(RadiansPerSecond) * drumRadiusMeters);
    inputs.appliedVoltage = voltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentSignal.getValueAsDouble();

    // inputs.positionRots = positionAngleSignal.getValue().in(Rotations);
    // inputs.velocityRotsPerSec = angularVelocitySignal.getValue().in(RotationsPerSecond);
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
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }
}
