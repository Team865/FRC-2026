package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends Pivot implements SysIdTestable {
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
  private final SysIdRoutine sysIdRoutine;

  public Turret(PivotIO pivotIO) {
    super("Shooter/Turret", pivotIO);

    pivotIO.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    pivotIO.setMotionProfile(maxVelocity.get(), maxAcceleration.get());
    pivotIO.updateInputs(inputsAutoLogged);

    sysIdRoutine =
        new SysIdBuilder(this, io::setVolts)
            .withDynamicStepVoltage(3.0)
            .withQuasistaticRampRate(0.2)
            .build();

    optimizeAngle(Degrees.of(0));
  }

  public Command lockOntoTarget(
      Supplier<Angle> relativeAngleSupplier, Supplier<Double> driveOmegaRadPerSecSupplier) {
    return this.runEnd(
        () -> {
          Angle rawAngle = relativeAngleSupplier.get();
          Angle optimizedAngle = optimizeAngle(rawAngle);

          Logger.recordOutput(
              "Turret/Requested Angle Degrees", relativeAngleSupplier.get().in(Degrees));
          Logger.recordOutput("Turret/Optimized Turret Angle Degrees", optimizedAngle.in(Degrees));

          io.setPosition(optimizedAngle.in(Radians));
        },
        () -> this.io.stop());
  }

  public Command lockOntoTarget(Supplier<Angle> relativeAngleSupplier) {
    return lockOntoTarget(relativeAngleSupplier, () -> 0.0);
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
    double referenceAngleRads = angle.in(Radians);
    referenceAngleRads =
        referenceAngleRads
            - TAU
                * Math.floor(
                    (referenceAngleRads + Math.PI) / (TAU)); // Limit it to within (-PI, PI)

    double positiveTargetAngleRads =
        (referenceAngleRads >= 0) ? referenceAngleRads : referenceAngleRads + TAU;
    double negativeTargetAngleRads =
        (referenceAngleRads <= 0) ? referenceAngleRads : referenceAngleRads - TAU;

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

    Logger.recordOutput(
        "Turret/PositionRots", Units.radiansToRotations(inputsAutoLogged.positionRads));
    Logger.recordOutput(
        "Turret/VelocityRotsPerSec", Units.radiansToRotations(inputsAutoLogged.velocityRadsPerSec));

    super.periodic();
  }

  @Override
  public SysIdRoutine getRoutine() {
    return sysIdRoutine;
  }
}
