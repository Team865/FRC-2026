package frc.robot.subsystems.extension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ControlSystemConstants;

public class ExtensionIOSim implements ExtensionIO {
  private double appliedVolts = 0.0;
  private Distance targetPosition = Meters.zero();
  private boolean voltageOverridesTarget = false;

  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));

  public ExtensionIOSim(DCMotor motor, ControlSystemConstants constants) {
    this.motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV, constants.kA), motor);

    setControlConstants(constants.kS, constants.kV, constants.kA, constants.kP, constants.kD);
    setMotionProfile(constants.maxVelocity.get(), constants.maxAcceleration.get());
  }

  private void setClampedVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setVolts(double volts) {
    setClampedVolts(volts);
    voltageOverridesTarget = true;
  }

  @Override
  public void setPosition(Distance position) {
    targetPosition = position;
    voltageOverridesTarget = false;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    voltageOverridesTarget = true;
  }

  @Override
  public void updateInputs(ExtensionIOInputsAutoLogged inputs) {
    if (!this.voltageOverridesTarget) {
      double positionMeters = motorSim.getAngularPositionRad();

      double pidEffort = pidController.calculate(positionMeters, targetPosition.in(Meters));
      double ffEffort = feedforwardController.calculate(pidEffort);

      setClampedVolts(pidEffort + ffEffort);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.020);

    inputs.connected = true;
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = inputs.supplyCurrentAmps;
    inputs.velocity = MetersPerSecond.of(motorSim.getAngularVelocityRadPerSec());
    inputs.position = Meters.of(motorSim.getAngularPositionRad());
    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    feedforwardController.setKs(kS);
    feedforwardController.setKa(kA);
    feedforwardController.setKv(kV);

    pidController.setP(kP);
    pidController.setD(kD);
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    pidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }
}
