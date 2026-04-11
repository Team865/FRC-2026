package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX masterTalon;
  private final TalonFX followerTalon;

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private AngularVelocity targetVelocity = RotationsPerSecond.zero();

  private final VoltageOut voltageRequest =
      new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final Debouncer masterConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> masterVelocity;
  private final StatusSignal<AngularVelocity> followerVelocity;

  private final StatusSignal<Voltage> masterAppliedVoltage;
  private final StatusSignal<Current> masterSupplyCurrent;
  private final StatusSignal<Current> masterStatorCurrent;

  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerStatorCurrent;

  @SuppressWarnings("removal")
  public FlywheelIOTalonFX() {
    masterTalon = new TalonFX(ShooterConstants.Flywheel.MASTER_CAN_ID, ShooterConstants.CANBUS);
    followerTalon = new TalonFX(ShooterConstants.Flywheel.FOLLOWER_CAN_ID, ShooterConstants.CANBUS);
    followerTalon.setControl(new Follower(masterTalon.getDeviceID(), MotorAlignmentValue.Opposed));

    ControlSystemConstants constants = ShooterConstants.Flywheel.SYSTEM_CONSTANTS;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEAR_RATIO;

    config.Slot0.kS = constants.kS;
    config.Slot0.kV = constants.kV;
    config.Slot0.kP = constants.kP;
    config.Slot0.kD = constants.kD;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 200.0;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 500.0;

    PhoenixUtil.tryUntilOk(5, () -> masterTalon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));

    position = masterTalon.getPosition();
    masterVelocity = masterTalon.getVelocity();
    followerVelocity = followerTalon.getVelocity();

    masterAppliedVoltage = masterTalon.getMotorVoltage();
    masterSupplyCurrent = masterTalon.getSupplyCurrent();
    masterStatorCurrent = masterTalon.getStatorCurrent();

    followerAppliedVoltage = followerTalon.getMotorVoltage();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerStatorCurrent = followerTalon.getStatorCurrent();

    masterTalon.optimizeBusUtilization();
    followerTalon.optimizeBusUtilization();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                masterVelocity,
                followerVelocity,
                masterAppliedVoltage,
                masterSupplyCurrent,
                masterStatorCurrent,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerStatorCurrent));
  }

  @Override
  public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
    inputs.masterConnected =
        masterConnectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position,
                    masterVelocity,
                    masterAppliedVoltage,
                    masterSupplyCurrent,
                    masterStatorCurrent)
                .isOK());

    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    followerAppliedVoltage,
                    followerVelocity,
                    followerSupplyCurrent,
                    followerStatorCurrent)
                .isOK());

    inputs.position = position.getValue();
    inputs.targetVelocity = targetVelocity;
    inputs.masterVelocity = masterVelocity.getValue();
    inputs.followerVelocity = followerVelocity.getValue();

    inputs.masterAppliedVoltage = masterAppliedVoltage.getValueAsDouble();
    inputs.masterSupplyCurrentAmps = masterSupplyCurrent.getValueAsDouble();
    inputs.masterStatorCurrentAmps = masterStatorCurrent.getValueAsDouble();

    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();

    // Logger.recordOutput(
    //     "Flywheel/VelocityRotsPerSec", inputs.masterVelocity.in(RotationsPerSecond));
    // Logger.recordOutput("Flywheel/PositionRots", inputs.position.in(Rotations));
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> masterTalon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerTalon.getConfigurator().apply(config));
  }

  @Override
  public void setVolts(double volts) {
    masterTalon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    masterTalon.setControl(velocityRequest.withVelocity(velocity));
    targetVelocity = velocity;
  }

  @Override
  public void stop() {
    masterTalon.setControl(neutralRequest);
  }
}
