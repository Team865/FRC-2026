package frc.robot.subsystems.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollersIOSim implements RollersIO {

  private final DCMotorSim sim;
  private final RollersConstants constants;

  private double appliedVoltage = 0.0;
  private double targetVelocityRadPerSec = 0.0;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController feedback = new PIDController(0.0, 0.0, 0.0);

  public RollersIOSim(DCMotor motor, double moi, RollersConstants constants) {
    this.constants = constants;

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, moi, constants.gearRatio()), motor);
  }

  @Override
  public void updateInputs(RollersIOInputsAutoLogged inputs) {
    sim.update(0.02);

    double motorTargetVel = targetVelocityRadPerSec / constants.gearRatio();

    double ffVolts = feedforward.calculate(motorTargetVel);
    double fbVolts = feedback.calculate(sim.getAngularVelocityRadPerSec(), motorTargetVel);

    setVolts(ffVolts + fbVolts);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad() * constants.gearRatio();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec() * constants.gearRatio();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
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
}
