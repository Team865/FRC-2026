package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ControlSystemContext;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim sim;

  private double appliedVoltage = 0.0;
  private double targetVelocityRadPerSec = 0.0;

  private SimpleMotorFeedforward feedforward;
  private PIDController feedback;

  public FlywheelIOSim() {
    ControlSystemContext baseGains = ShooterConstants.Flywheel.getConstants();

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(baseGains.kV(), baseGains.kA()),
            DCMotor.getKrakenX60(2));

    feedforward = new SimpleMotorFeedforward(baseGains.kS(), baseGains.kV());

    feedback = new PIDController(baseGains.kP(), 0.0, baseGains.kD());
  }

  @Override
  public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
    sim.update(0.02);

    double motorTargetVel = targetVelocityRadPerSec / ShooterConstants.Flywheel.GEAR_RATIO;

    double ffVolts = feedforward.calculate(motorTargetVel);
    double fbVolts = feedback.calculate(sim.getAngularVelocityRadPerSec(), motorTargetVel);

    setVolts(ffVolts + fbVolts);

    inputs.positionRads = sim.getAngularPositionRad() * ShooterConstants.Flywheel.GEAR_RATIO;
    inputs.velocityRadsPerSec =
        sim.getAngularVelocityRadPerSec() * ShooterConstants.Flywheel.GEAR_RATIO;

    inputs.masterConnected = true;
    inputs.masterAppliedVoltage = appliedVoltage;
    inputs.masterSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.masterStatorCurrentAmps = inputs.masterSupplyCurrentAmps;

    inputs.followerConnected = true;
    inputs.followerAppliedVoltage = inputs.masterAppliedVoltage;
    inputs.followerSupplyCurrentAmps = inputs.masterSupplyCurrentAmps;
    inputs.followerStatorCurrentAmps = inputs.masterStatorCurrentAmps;
  }

  @Override
  public void setVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    targetVelocityRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setControlConstants(double kS, double kV, double kP, double kD) {
    feedforward = new SimpleMotorFeedforward(kS, kV);
    feedback.setPID(kP, 0.0, kD);
  }

  @Override
  public void stop() {
    targetVelocityRadPerSec = 0;
  }
}
