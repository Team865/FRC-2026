package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends Pivot {
  private final ControlSystemContext gains = ShooterConstants.Turret.getGains();
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Turret/kS", gains.kS());
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/Turret/kV", gains.kV());
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/Turret/kA", gains.kA());
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Turret/kP", gains.kP());
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Turret/kD", gains.kD());

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Shooter/Turret/MaxVelocity", gains.maxVelocity().get());
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Shooter/Turret/MaxAcceleration", gains.maxAcceleration().get());

  private final double FULL_CIRCLE_RADS = Math.PI * 2;

  public Turret(PivotIO pivotIO) {
    super("Shooter/Turret", pivotIO);
  }

  public Command lockOntoTarget(
      Supplier<Angle> relativeAngleSupplier, Supplier<Double> driveOmegaRadPerSecSupplier) {
    return this.runEnd(
        () -> {
          double relativeAngleRads = relativeAngleSupplier.get().in(Radians);
          Logger.recordOutput("Turret/RelativeAngleOffset", relativeAngleRads);

          // Calculate the nearest plausible absolute angle
          double positiveTargetAngleRads =
              (relativeAngleRads >= 0) ? relativeAngleRads : FULL_CIRCLE_RADS + relativeAngleRads;
          double negativeTargetAngleRads =
              (relativeAngleRads < 0) ? relativeAngleRads : -FULL_CIRCLE_RADS + relativeAngleRads;

          double absoluteAngleRads;

          if (positiveTargetAngleRads > ShooterConstants.Turret.MAX_ANGLE_RADS) {
            absoluteAngleRads = negativeTargetAngleRads;
          } else if (negativeTargetAngleRads < ShooterConstants.Turret.MIN_ANGLE_RADS) {
            absoluteAngleRads = positiveTargetAngleRads;
          } else {
            double errorFromPositiveAngle =
                Math.abs(positiveTargetAngleRads - inputsAutoLogged.positionRads);
            double errorFromNegativeAngle =
                Math.abs(negativeTargetAngleRads - inputsAutoLogged.positionRads);
            absoluteAngleRads =
                (errorFromPositiveAngle < errorFromNegativeAngle)
                    ? positiveTargetAngleRads
                    : negativeTargetAngleRads;
          }

          Logger.recordOutput("Turret/PositiveTarget", positiveTargetAngleRads);
          Logger.recordOutput("Turret/NegativeTarget", negativeTargetAngleRads);
          Logger.recordOutput("Turret/AbsoluteTargetAngle", absoluteAngleRads);

          this.io.setPositionWithExtraOmega(absoluteAngleRads, -driveOmegaRadPerSecSupplier.get());
        },
        () -> {
          this.io.stop();
        });
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id,
        (constants) ->
            this.io.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        kS,
        kV,
        kA,
        kP,
        kD);
    LoggedTunableNumber.ifChanged(
        id,
        (constants) -> this.io.setMotionProfile(constants[0], constants[1]),
        maxVelocity,
        maxAcceleration);

    super.periodic();
  }
}
