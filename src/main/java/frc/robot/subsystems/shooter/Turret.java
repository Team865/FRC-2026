package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends Pivot {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Turret/kS", ShooterConstants.Turret.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Turret/kV", ShooterConstants.Turret.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Turret/kA", ShooterConstants.Turret.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Turret/kP", ShooterConstants.Turret.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Turret/kD", ShooterConstants.Turret.SYSTEM_CONSTANTS.kD);

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Shooter/Turret/MaxVelocity", ShooterConstants.Turret.SYSTEM_CONSTANTS.maxVelocity.get());
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Shooter/Turret/MaxAcceleration",
          ShooterConstants.Turret.SYSTEM_CONSTANTS.maxAcceleration.get());

  private static final double TAU = 2 * Math.PI;

  public Turret(PivotIO pivotIO) {
    super("Shooter/Turret", pivotIO);

    pivotIO.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    pivotIO.setMotionProfile(maxVelocity.get(), maxAcceleration.get());
    pivotIO.updateInputs(inputsAutoLogged);
  }

  public Command lockOntoTarget(
      Supplier<Angle> relativeAngleSupplier, Supplier<Double> driveOmegaRadPerSecSupplier) {
    return this.runEnd(
        () -> {
          Angle optimizedAngle = optimizeAngle(relativeAngleSupplier.get());

          if (MathUtil.isNear(inputsAutoLogged.positionRads, optimizedAngle.in(Radians), 0.1)) {
            io.stop();
            return;
          }
          Logger.recordOutput(
              "Turret/Requested Angle Degrees", relativeAngleSupplier.get().in(Degrees));
          Logger.recordOutput("Turret/Optimized Turret Angle Degrees", optimizedAngle.in(Degrees));

          this.io.setPosition(optimizedAngle.in(Radians));
        },
        () -> this.io.stop());
  }

  /**
   * Calculates the closest angle the turret can go to that would reach the given angle within the
   * turret's physical limits.
   *
   * <p>For example, if the turret's:
   *
   * <ul>
   *   <li>Minimum Angle: -200 degrees
   *   <li>Maximum Angle: +200 degrees
   *   <li>Current Angle: 180 degrees
   * </ul>
   *
   * If you call optimizeAngle(-170 degrees), it will instead give 190 degrees as it achieves the
   * same orientation while taking a much shorter path.<br>
   * <br>
   * If you call optimizeAngle(-150 degrees), it will return -150 degrees as although 210 degrees is
   * a shorter path, it is outside the turret's limits.
   *
   * @param angle The target angle
   * @return The optimized angle
   */
  public Angle optimizeAngle(Angle angle) {
    double referenceAnglerads = angle.in(Radians);
    referenceAnglerads =
        referenceAnglerads
            - TAU
                * Math.floor(
                    (referenceAnglerads + Math.PI) / (TAU)); // Limit it to within (-PI, PI)

    double positiveTargetAngleRads =
        (referenceAnglerads > 0) ? referenceAnglerads : referenceAnglerads + TAU;
    double negativeTargetAngleRads =
        (referenceAnglerads < 0) ? referenceAnglerads : referenceAnglerads - TAU;

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

    return Radians.of(absoluteAngleRads);
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

    Logger.recordOutput("Encoder Mechanism Rotations", io.getEncoderAngle());

    super.periodic();
  }
}
