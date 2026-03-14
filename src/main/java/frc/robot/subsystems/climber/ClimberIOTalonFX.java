package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Hertz;

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
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX talon;
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

  @SuppressWarnings("removal")
  public ClimberIOTalonFX() {
    talon = new TalonFX(ClimberConstants.CAN_ID, ClimberConstants.CANBUS);

    ControlSystemConstants constants = ClimberConstants.SYSTEM_CONSTANTS;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio =
        ClimberConstants.GEAR_RATIO / (2 * Math.PI * ClimberConstants.DRUM_RADIUS_METERS);

    config.Slot0.kG = constants.kG;
    config.Slot0.kS = constants.kS;
    config.Slot0.kV = constants.kV;
    config.Slot0.kA = constants.kA;
    config.Slot0.kP = constants.kP;
    config.Slot0.kD = constants.kD;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    positionMetersSignal = talon.getPosition();
    velocityMetersPerSecSignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    supplyCurrentSignal = talon.getSupplyCurrent();
    statorCurrentSignal = talon.getStatorCurrent();

    talon.optimizeBusUtilization();
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                Hertz.of(50.0),
                positionMetersSignal,
                velocityMetersPerSecSignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal));
  }

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    boolean refreshSucceeded =
        BaseStatusSignal.refreshAll(
                positionMetersSignal,
                velocityMetersPerSecSignal,
                voltageSignal,
                supplyCurrentSignal,
                statorCurrentSignal)
            .isOK();
    inputs.connected = connectedDebouncer.calculate(refreshSucceeded);

    if (!refreshSucceeded) return;

    inputs.appliedVoltage = voltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();
    inputs.positionMeters = positionMetersSignal.getValueAsDouble();
    inputs.velocityMetersPerSec = velocityMetersPerSecSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double positionMeters) {
    talon.setControl(positionRequest.withPosition(positionMeters));
  }

  @Override
  public void setControlGains(double kG, double kS, double kV, double kA, double kP, double kD) {
    config.Slot0.kG = kG;
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

  @Override
  public void stop() {
    talon.setControl(neutralRequest);
  }
}
