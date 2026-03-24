package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.pivot.AbsoluteEncoderIO;
import frc.robot.subsystems.pivot.AbsoluteEncoderInputsAutoLogged;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends Pivot implements SysIdTestable {
  private static final double TAU = 2 * Math.PI;

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

  private final AbsoluteEncoderIO encoderIO;
  private final AbsoluteEncoderInputsAutoLogged encoderInputs =
      new AbsoluteEncoderInputsAutoLogged();

  private final Alert encoderDisconnectedAlert =
      new Alert("Turret encoder disconnected.", AlertType.kError);
  private final SysIdRoutine sysIdRoutine;

  public Turret(PivotIO pivotIO, AbsoluteEncoderIO encoderIO) {
    super("Shooter/Turret", pivotIO);
    this.encoderIO = encoderIO;
    encoderIO.updateInputs(encoderInputs);

    pivotIO.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    pivotIO.setMotionProfile(maxVelocity.get(), maxAcceleration.get());
    pivotIO.updateInputs(inputs);

    // try (Alert failedReseedAlert =
    //     new Alert("Turret could not be seeded from encoder.", AlertType.kWarning)) {
    //   failedReseedAlert.set(!pivotIO.seedPosition(encoderInputs.position));
    // }

    sysIdRoutine =
        new SysIdBuilder(this, io::setVolts)
            .withDynamicStepVoltage(3.0)
            .withQuasistaticRampRate(0.2)
            .build();

    optimizeAngle(Degrees.of(0));
  }

  public Command lockOntoTarget(
      Supplier<Angle> relativeAngleSupplier, Supplier<AngularVelocity> driveOmegaSupplier) {
    return this.runEnd(
        () -> {
          Angle rawAngle = relativeAngleSupplier.get();
          Angle optimizedAngle = optimizeAngle(rawAngle);

          // // Apply deadzone to reduce how much noise affects tracking
          // if (inputs.targetPosition.isNear(optimizedAngle, ShooterConstants.Turret.DEADZONE)) {
          //   optimizedAngle = inputs.targetPosition;
          // }

          Logger.recordOutput(
              "Turret/Requested Angle Degrees", relativeAngleSupplier.get().in(Degrees));
          Logger.recordOutput("Turret/Optimized Turret Angle Degrees", optimizedAngle.in(Degrees));

          double currentPositionRads = inputs.position.in(Radians);

          if (currentPositionRads
                  < (ShooterConstants.Turret.MIN_ANGLE_RADS + Units.degreesToRadians(10))
              || currentPositionRads
                  > (ShooterConstants.Turret.MAX_ANGLE_RADS + Units.degreesToRadians(10))) {
            io.setPosition(optimizedAngle);
          } else {
            io.setPositionWithExtraOmega(optimizedAngle, driveOmegaSupplier.get());
          }
        },
        () -> this.io.stop());
  }

  public Command lockOntoTarget(Supplier<Angle> relativeAngleSupplier) {
    return lockOntoTarget(relativeAngleSupplier, () -> RotationsPerSecond.zero());
  }

  public Command manualControl(DoubleSupplier gainSupplier) {
    Command command =
        new Command() {
          private Angle currentTargetPosition;

          @Override
          public void initialize() {
            currentTargetPosition = getOrientation();
          }

          @Override
          public void execute() {
            double gain = MathUtil.applyDeadband(gainSupplier.getAsDouble(), 0.1);

            if (gain == 0.0) return;

            currentTargetPosition =
                currentTargetPosition.plus(ShooterConstants.Turret.MANUAL_CONTROL_RATE.times(gain));
            Angle optimizedAngle = optimizeAngle(currentTargetPosition);

            io.setPosition(optimizedAngle);
          }
        };

    command.addRequirements(this);

    return command;
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
    double currentPositionRads = inputs.position.in(Radians);

    if (positiveTargetAngleRads > ShooterConstants.Turret.MAX_ANGLE_RADS) {
      absoluteAngleRads = negativeTargetAngleRads;
    } else if (negativeTargetAngleRads < ShooterConstants.Turret.MIN_ANGLE_RADS) {
      absoluteAngleRads = positiveTargetAngleRads;
    } else {
      double errorFromPositiveAngle = Math.abs(positiveTargetAngleRads - currentPositionRads);
      double errorFromNegativeAngle = Math.abs(negativeTargetAngleRads - currentPositionRads);
      absoluteAngleRads =
          (errorFromPositiveAngle < errorFromNegativeAngle)
              ? positiveTargetAngleRads
              : negativeTargetAngleRads;
    }

    return Radians.of(absoluteAngleRads);
  }

  public Trigger canShoot() {
    return new Trigger(
        () ->
            inputs.targetPosition.isNear(
                inputs.position, ShooterConstants.Turret.SHOOTING_TOLERANCE));
  }

  @Override
  public void periodic() {
    int id = hashCode();

    // LoggedTunableNumber.ifChanged(
    //     id,
    //     (constants) ->
    //         this.io.setControlConstants(
    //             constants[0], constants[1], constants[2], constants[3], constants[4]),
    //     kS,
    //     kV,
    //     kA,
    //     kP,
    //     kD);
    // LoggedTunableNumber.ifChanged(
    //     id,
    //     (constants) -> this.io.setMotionProfile(constants[0], constants[1]),
    //     maxVelocity,
    //     maxAcceleration);

    super.periodic();

    encoderIO.updateInputs(encoderInputs);
    // encoderDisconnectedAlert.set(!encoderInputs.connected);
    Logger.processInputs("Shooter/Turret/AbsoluteEncoder", encoderInputs);
  }

  @Override
  public SysIdRoutine getRoutine() {
    return sysIdRoutine;
  }
}
