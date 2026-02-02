package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import frc.robot.Constants.ControlSystemContext;
import frc.robot.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {
  public final TalonFX motor;
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

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public record PivotTalonFXConstants(
      ControlSystemContext controlSystemConstants, boolean clockwisePositive, double gearRatio) {}

  public PivotIOTalonFX(int canId, String canBus, PivotTalonFXConstants constants) {
    motor = new TalonFX(canId, canBus);

    ControlSystemContext controlSystemConstants = constants.controlSystemConstants();

    motorConfig.MotorOutput.Inverted =
        constants.clockwisePositive
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.Feedback.SensorToMechanismRatio = constants.gearRatio;

    motorConfig.Slot0.kV = controlSystemConstants.kV();
    motorConfig.Slot0.kA = controlSystemConstants.kA();
    motorConfig.Slot0.kS = controlSystemConstants.kS();

    motorConfig.Slot0.kP = controlSystemConstants.kP();
    motorConfig.Slot0.kD = controlSystemConstants.kD();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = controlSystemConstants.maxVelocity().get();
    motorConfig.MotionMagic.MotionMagicAcceleration =
        controlSystemConstants.maxAcceleration().get();

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    supplyCurrentSignal = motor.getSupplyCurrent();
    statorCurrentSignal = motor.getStatorCurrent();
  }

  @Override
  public void setVolts(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double angleRads) {
    motor.setControl(positionRequest.withPosition(Radians.of(angleRads)));
  }

  @Override
  public void setPositionWithExtraOmega(double angleRads, double omegaRadPerSec) {
    double omegaRPS = omegaRadPerSec / (2 * Math.PI);

    motor.setControl(
        positionRequest
            .withPosition(Radians.of(omegaRadPerSec))
            .withFeedForward(motorConfig.Slot0.kV * omegaRPS));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
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

    inputs.positionRads = positionSignal.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocitySignal.getValue().in(RadiansPerSecond);
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

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = maxAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig));
  }
}
