package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ControlSystemConstants;

public class PivotIOSim implements PivotIO {
  private double appliedVolts = 0.0;
  private Angle targetAngle = Rotations.zero();
  private boolean voltageOverridesTarget = false;
  private AngularVelocity extraOmega = RotationsPerSecond.zero();

  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  public PivotIOSim(DCMotor motor, ControlSystemConstants constants) {
    this.motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.kV, constants.kA), motor);

    motorSim.setAngle(0.0);

    setControlConstants(constants.kS, constants.kV, constants.kA, constants.kP, constants.kD);
    setMotionProfile(constants.maxVelocity.get(), constants.maxAcceleration.get());
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
  public void setPosition(Angle angle) {
    this.targetAngle = angle;
    this.voltageOverridesTarget = false;
  }

  @Override
  public void setPositionWithExtraOmega(Angle angle, AngularVelocity omega) {
    setPosition(angle);
    this.extraOmega = omega;
  }

  @Override
  public void stop() {
    this.appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    if (!this.voltageOverridesTarget) {
      double angRad = motorSim.getAngularPositionRad();

      double pidOutput = this.pidController.calculate(angRad, this.targetAngle.in(Radians));
      double feedforwardOutput =
          this.feedforwardController.calculate(pidOutput + extraOmega.in(RadiansPerSecond));

      this.setClampedVolts(pidOutput + feedforwardOutput);
    }

    this.motorSim.setInputVoltage(this.appliedVolts);
    this.motorSim.update(0.020);

    inputs.connected = true;
    inputs.appliedVoltage = this.appliedVolts;
    inputs.supplyCurrentAmps = this.motorSim.getCurrentDrawAmps();
    inputs.statorCurrentAmps = inputs.supplyCurrentAmps;
    inputs.torqueCurrent = inputs.supplyCurrentAmps;
    inputs.velocity = this.motorSim.getAngularVelocity();
    inputs.targetPosition = targetAngle;
    inputs.position = this.motorSim.getAngularPosition();
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

  @Override
  public boolean seedPosition(Angle position) {
    this.motorSim.setAngle(position.in(Radians));
    return true;
  }
}
