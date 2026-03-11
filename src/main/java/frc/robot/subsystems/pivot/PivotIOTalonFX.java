package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {
  public final TalonFX talon;
  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final VoltageOut voltageRequest =
      new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;

  private Angle targetAngle = Rotations.zero();

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  @SuppressWarnings("removal")
  public PivotIOTalonFX(int canId, String canBus, PivotSpecifications specs) {
    talon = new TalonFX(canId, canBus);

    motorConfig.MotorOutput.Inverted =
        specs.clockwisePositive()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = specs.statorCurrentLimit();
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = specs.supplyCurrentLimit();

    motorConfig.Feedback.SensorToMechanismRatio = specs.gearRatio();
    motorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(0));

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    supplyCurrentSignal = talon.getSupplyCurrent();
    statorCurrentSignal = talon.getStatorCurrent();

    talon.optimizeBusUtilization();
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                Hertz.of(50.0),
                positionSignal,
                velocitySignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal));
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle angle) {
    talon.setControl(positionRequest.withPosition(angle));
    this.targetAngle = angle;
  }

  @Override
  public void setPositionWithExtraOmega(Angle angle, AngularVelocity omega) {
    double omegaRPS = omega.in(RotationsPerSecond);

    this.targetAngle = angle;
    talon.setControl(
        positionRequest
            // .withPosition(angle)
            .withFeedForward(motorConfig.Slot0.kV * omegaRPS));
  }

  @Override
  public void stop() {
    talon.setControl(neutralRequest);
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    positionSignal,
                    velocitySignal,
                    voltageSignal,
                    supplyCurrentSignal,
                    statorCurrentSignal)
                .isOK());

    inputs.targetPosition = targetAngle;
    inputs.position = positionSignal.getValue();
    inputs.velocity = velocitySignal.getValue();
    inputs.appliedVoltage = voltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(motorConfig));
  }

  @Override
  public boolean seedPosition(Angle position) {
    return PhoenixUtil.tryUntilOk(5, () -> talon.setPosition(position, 0.5));
  }
}
