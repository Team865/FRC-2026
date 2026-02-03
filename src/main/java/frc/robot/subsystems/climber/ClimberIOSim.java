package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
  private double targetPosition = 0.0;
  private double appliedVolts = 0.0;
  private boolean isUsingTarget = false;

  private final ElevatorSim physicsSim =
      new ElevatorSim(
          ClimberConstants.getConstants().kV(),
          ClimberConstants.getConstants().kA(),
          DCMotor.getKrakenX60(1),
          0,
          5,
          true,
          0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          ClimberConstants.getConstants().kP(),
          0,
          ClimberConstants.getConstants().kD(),
          new Constraints(0, 0));
  private final ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(
          ClimberConstants.getConstants().kS(),
          ClimberConstants.getConstants().kG(),
          ClimberConstants.getConstants().kV(),
          ClimberConstants.getConstants().kA());

  @Override
  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    if (this.isUsingTarget) {
      double pidOutput =
          pidController.calculate(physicsSim.getPositionMeters(), this.targetPosition);
      double feedforwardOutput = feedforwardController.calculate(pidOutput);

      this.appliedVolts = pidOutput + feedforwardOutput;
    }

    physicsSim.setInputVoltage(appliedVolts);
    physicsSim.update(0.020);

    inputs.connected = true;
    inputs.appliedVoltage = appliedVolts;
    inputs.positionMeters = physicsSim.getPositionMeters();
    inputs.velocityMetersPerSec = physicsSim.getVelocityMetersPerSecond();
    inputs.supplyCurrentAmps = physicsSim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = inputs.supplyCurrentAmps;
  }

  @Override
  public void setVoltage(double volts) {
    this.appliedVolts = volts;
    this.isUsingTarget = false;
  }

  @Override
  public void setPosition(double positionMeters) {
    this.targetPosition = positionMeters;
    this.isUsingTarget = true;
  }

  @Override
  public void setControlGains(double kG, double kS, double kV, double kA, double kP, double kD) {
    this.pidController.setPID(kP, 0, kD);
    this.feedforwardController.setKv(kV);
    this.feedforwardController.setKa(kA);
    this.feedforwardController.setKv(kV);
    this.feedforwardController.setKs(kS);
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    this.pidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }
}
