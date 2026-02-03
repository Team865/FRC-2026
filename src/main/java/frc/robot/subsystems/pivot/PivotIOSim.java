package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ControlSystemContext;

public class PivotIOSim implements PivotIO {
  private double appliedVolts = 0.0;
  private double targetAngleRads = 0.0;
  private boolean voltageOverridesTarget = false;
  private double extraOmegaRadPerSec = 0.0;

  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  public PivotIOSim(DCMotor motor, ControlSystemContext constants) {
    this.motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV(), constants.kA()), motor);

    setControlConstants(
        constants.kS(), constants.kV(), constants.kA(), constants.kP(), constants.kD());
    setMotionProfile(constants.maxVelocity().get(), constants.maxAcceleration().get());
  }

  @Override
  public void setVolts(double volts) {
    this.setClampedVolts(volts);
    this.voltageOverridesTarget = true;
  }

  private void setClampedVolts(double volts) {
    this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setPosition(double angleRads) {
    this.targetAngleRads = angleRads;
    this.voltageOverridesTarget = false;
  }

  @Override
  public void setPositionWithExtraOmega(double angleRads, double omegaRadPerSec) {
    setPosition(angleRads);
    this.extraOmegaRadPerSec = omegaRadPerSec;
  }

  @Override
  public void stop() {
    this.appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    if (!this.voltageOverridesTarget) {
      double angRad = motorSim.getAngularPositionRad();

      double pidOutput = this.pidController.calculate(angRad, this.targetAngleRads);
      double feedforwardOutput =
          this.feedforwardController.calculate(pidOutput + extraOmegaRadPerSec);

      this.setClampedVolts(pidOutput + feedforwardOutput);
    }

    this.motorSim.setInputVoltage(this.appliedVolts);
    this.motorSim.update(0.020);

    inputs.connected = true;
    inputs.appliedVoltage = this.appliedVolts;
    inputs.supplyCurrentAmps = this.motorSim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = inputs.supplyCurrentAmps;
    inputs.torqueCurrent = inputs.supplyCurrentAmps;
    inputs.velocityRadsPerSec = this.motorSim.getAngularVelocityRadPerSec();
    inputs.positionRads = this.motorSim.getAngularPositionRad();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    this.feedforwardController.setKa(kA);
    this.feedforwardController.setKs(kS);
    this.feedforwardController.setKv(kV);

    this.pidController.setPID(kP, 0.0, kD);
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    this.pidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }
}
