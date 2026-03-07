package frc.robot.subsystems.rollers;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ControlSystemConstants;

public class RollersIOSim implements RollersIO {

  private final DCMotorSim sim;
  private final RollersSpecifications specs;

  private double appliedVoltage = 0.0;
  private AngularVelocity targetVelocity = RadiansPerSecond.zero();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController feedback = new PIDController(0.0, 0.0, 0.0);

  public RollersIOSim(DCMotor motor, double moi, RollersSpecifications specs) {
    this.specs = specs;

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, moi, 1 / specs.gearRatio()), motor);
  }

  public RollersIOSim(
      DCMotor motor, ControlSystemConstants constants, RollersSpecifications specs) {
    this.specs = specs;

    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV, constants.kA), motor);
  }

  @Override
  public void updateInputs(RollersIOInputsAutoLogged inputs) {
    sim.update(0.02);

    double motorTargetVel = targetVelocity.in(RadiansPerSecond) * specs.gearRatio();
    double ffVolts = feedforward.calculate(motorTargetVel);
    double fbVolts = feedback.calculate(sim.getAngularVelocityRadPerSec(), motorTargetVel);

    setVolts(ffVolts + fbVolts);

    inputs.connected = true;
    inputs.position = sim.getAngularPosition().times(specs.gearRatio());
    inputs.velocity = sim.getAngularVelocity().times(specs.gearRatio());
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  @Override
  public void setLinearVelocity(LinearVelocity velocity) {
    targetVelocity = RadiansPerSecond.of(velocity.in(MetersPerSecond) / specs.rollerRadiusMeters());
  }

  @Override
  public void stop() {
    targetVelocity = RotationsPerSecond.zero();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    feedforward = new SimpleMotorFeedforward(kS, kV);
    feedback.setPID(kP, 0.0, kD);
  }
}
