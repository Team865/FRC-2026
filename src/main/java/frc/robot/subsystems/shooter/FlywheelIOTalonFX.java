package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import frc.robot.Constants.ControlSystemContext;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX masterMotor;
  private final TalonFX followerMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut voltageRequest =
      new VoltageOut(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0.0).withUpdateFreqHz(50.0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final Debouncer masterConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> masterAppliedVoltage;
  private final StatusSignal<Current> masterSupplyCurrent;
  private final StatusSignal<Current> masterStatorCurrent;

  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerStatorCurrent;

  @SuppressWarnings("removal")
  public FlywheelIOTalonFX() {
    masterMotor =
        new TalonFX(ShooterConstants.Flywheel.MASTER_CAN_ID, ShooterConstants.Flywheel.CANBUS);
    followerMotor =
        new TalonFX(ShooterConstants.Flywheel.FOLLOWER_CAN_ID, ShooterConstants.Flywheel.CANBUS);
    followerMotor.setControl(new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    ControlSystemContext constants = ShooterConstants.Flywheel.getConstants();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEAR_RATIO;

    config.Slot0.kS = constants.kS();
    config.Slot0.kV = constants.kV();
    config.Slot0.kP = constants.kP();
    config.Slot0.kD = constants.kD();

    PhoenixUtil.tryUntilOk(5, () -> masterMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config));

    position = masterMotor.getPosition();
    velocity = masterMotor.getVelocity();

    masterAppliedVoltage = masterMotor.getMotorVoltage();
    masterSupplyCurrent = masterMotor.getSupplyCurrent();
    masterStatorCurrent = masterMotor.getStatorCurrent();

    followerAppliedVoltage = followerMotor.getMotorVoltage();
    followerSupplyCurrent = followerMotor.getSupplyCurrent();
    followerStatorCurrent = followerMotor.getStatorCurrent();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
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
                    velocity,
                    masterAppliedVoltage,
                    masterSupplyCurrent,
                    masterStatorCurrent)
                .isOK());

    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    followerAppliedVoltage, followerSupplyCurrent, followerStatorCurrent)
                .isOK());

    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);

    inputs.masterAppliedVoltage = masterAppliedVoltage.getValueAsDouble();
    inputs.masterSupplyCurrentAmps = masterSupplyCurrent.getValueAsDouble();
    inputs.masterStatorCurrentAmps = masterStatorCurrent.getValueAsDouble();

    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kP, double kD) {
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> masterMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config));
  }

  @Override
  public void setVolts(double volts) {
    masterMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    masterMotor.setControl(velocityRequest.withVelocity(velocityRadPerSec));
  }

  @Override
  public void stop() {
    masterMotor.setControl(neutralRequest);
  }
}
