package frc.robot.subsystems.climber;

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

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest =
      new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  // Type system abuse, the Angles correspond to linear (meters), not angle
  private final StatusSignal<Angle> positionMetersSignal;
  private final StatusSignal<AngularVelocity> velocityMetersPerSecSignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ClimberIOTalonFX() {
    motor = new TalonFX(ClimberConstants.CAN_ID, ClimberConstants.CANBUS);

    ControlSystemContext constants = ClimberConstants.getConstants();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio =
        ClimberConstants.GEAR_RATIO / (2 * Math.PI * ClimberConstants.DRUM_RADIUS_METERS);

    config.Slot0.kG = constants.kG();
    config.Slot0.kS = constants.kS();
    config.Slot0.kV = constants.kV();
    config.Slot0.kA = constants.kA();
    config.Slot0.kP = constants.kP();
    config.Slot0.kD = constants.kD();

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    positionMetersSignal = motor.getPosition();
    velocityMetersPerSecSignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    supplyCurrentSignal = motor.getSupplyCurrent();
    statorCurrentSignal = motor.getStatorCurrent();
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    positionMetersSignal,
                    velocityMetersPerSecSignal,
                    voltageSignal,
                    supplyCurrentSignal,
                    statorCurrentSignal)
                .isOK());

    inputs.appliedVoltage = voltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.positionMeters = positionMetersSignal.getValueAsDouble();
    inputs.velocityMetersPerSec = velocityMetersPerSecSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double positionMeters) {
    motor.setControl(positionRequest.withPosition(positionMeters));
  }

  @Override
  public void setControlGains(double kG, double kS, double kV, double kA, double kP, double kD) {
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity;
    config.MotionMagic.MotionMagicAcceleration = maxAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }
}
