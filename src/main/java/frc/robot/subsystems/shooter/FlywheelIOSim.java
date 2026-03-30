package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ControlSystemConstants;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim sim;

  private double appliedVoltage = 0.0;
  private AngularVelocity targetVelocity = RotationsPerSecond.zero();

  private SimpleMotorFeedforward feedforward;
  private PIDController feedback;

  public FlywheelIOSim() {
    ControlSystemConstants constants = ShooterConstants.Flywheel.SYSTEM_CONSTANTS;

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(constants.kV, constants.kA),
            DCMotor.getKrakenX60(2));

    feedforward = new SimpleMotorFeedforward(constants.kS, constants.kV);

    feedback = new PIDController(constants.kP, 0.0, constants.kD);
  }

  @Override
  public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
    sim.update(0.02);

    double motorTargetVel =
        targetVelocity.in(RadiansPerSecond) / ShooterConstants.Flywheel.GEAR_RATIO;

    double ffVolts = feedforward.calculate(motorTargetVel);
    double fbVolts = feedback.calculate(sim.getAngularVelocityRadPerSec(), motorTargetVel);

    setVolts(ffVolts + fbVolts);

    inputs.position = sim.getAngularPosition().times(ShooterConstants.Flywheel.GEAR_RATIO);
    inputs.masterVelocity = sim.getAngularVelocity().times(ShooterConstants.Flywheel.GEAR_RATIO);

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
  public void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    feedforward = new SimpleMotorFeedforward(kS, kV);
    feedback.setPID(kP, 0.0, kD);
  }

  @Override
  public void stop() {
    targetVelocity = RotationsPerSecond.zero();
  }
}
