package frc.robot.subsystems.serializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SerializerIOSim implements SerializerIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0;
  private double targetVelocityRadPerSec = 0;

  private SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.2, 0.01);
  private PIDController feedbackController = new PIDController(5, 0, 0);

  public SerializerIOSim(DCMotor motor, double moi) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, moi, SerializerConstants.GEAR_RATIO), motor);
  }

  @Override
  public void updateInputs(SerializerIOInputsAutoLogged inputs) {
    sim.update(0.02);

    double motorTargetVel = targetVelocityRadPerSec / SerializerConstants.GEAR_RATIO;
    double ffVolts = feedforwardController.calculate(motorTargetVel);
    double fbVolts =
        feedbackController.calculate(sim.getAngularVelocityRadPerSec(), motorTargetVel);
    setVolts(ffVolts + fbVolts);
    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad() * SerializerConstants.GEAR_RATIO;
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec() * SerializerConstants.GEAR_RATIO;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    targetVelocityRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setControlConstants(double kS, double kV, double kP, double kD) {
    feedbackController.setPID(kP, 0, kD);
    feedforwardController = new SimpleMotorFeedforward(kS, kV);
  }
}
