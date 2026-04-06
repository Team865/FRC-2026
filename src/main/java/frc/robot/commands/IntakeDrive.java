package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;

public class IntakeDrive extends Command {
  private static final double DEADBAND = 0.1;
  private static final double MAX_SLOW_RATE = 0.1;

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final ProfiledPIDController angleController;

  private final double directionSwitchingThreshold = -0.35;

  private double previousJoystickLinMagnitude = 0.0;
  private Rotation2d previousHeading = Rotation2d.kZero;

  public IntakeDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double ANGLE_KP,
      double ANGLE_KD,
      double ANGLE_MAX_VELOCITY,
      double ANGLE_MAX_ACCELERATION) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    // Create PID controller
    angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    super.initialize();

    this.previousJoystickLinMagnitude = 0.0;
    this.previousHeading = Rotation2d.kZero;
  }

  @Override
  public void execute() {
    Rotation2d currentDriveRotation = drive.getRotation();
    boolean isFlipped = AllianceFlipUtil.shouldFlip();

    // Get linear velocity
    double linearX = xSupplier.getAsDouble();
    double linearY = ySupplier.getAsDouble();

    double targetLinearVelocityMagnitude =
        DriveCommands.getLinearVelocityMagnitude(linearX, linearY);
    Rotation2d targetHeading = new Rotation2d(Math.atan2(linearY, linearX));

    double magDiffFromLastUpdate = targetLinearVelocityMagnitude - previousJoystickLinMagnitude;

    Translation2d initalJoystickTarget =
        new Translation2d(targetLinearVelocityMagnitude, targetHeading);
    Translation2d previousJoystickTarget =
        new Translation2d(previousJoystickLinMagnitude, previousHeading);

    if (targetLinearVelocityMagnitude > 0) {
      previousHeading = targetHeading;
    }

    // Slowing down
    if ((initalJoystickTarget.dot(previousJoystickTarget) < 0.0)
        && (initalJoystickTarget.getDistance(previousJoystickTarget) > MAX_SLOW_RATE)) {
      Translation2d delta = initalJoystickTarget.minus(previousJoystickTarget);

      delta = delta.div(delta.getNorm()); // Normalize the vector

      Translation2d clampedJoysticks = previousJoystickTarget.plus(delta.times(MAX_SLOW_RATE));

      targetLinearVelocityMagnitude = clampedJoysticks.getNorm();
      // if(targetLinearVelocityMagnitude > 0)
      previousHeading = clampedJoysticks.getAngle();
    } else if (magDiffFromLastUpdate < 0) {
      // Clamp acceleration
      targetLinearVelocityMagnitude =
          previousJoystickLinMagnitude + Math.max(magDiffFromLastUpdate, -MAX_SLOW_RATE);
    }

    previousJoystickLinMagnitude = targetLinearVelocityMagnitude;

    Translation2d linearVelocity =
        new Translation2d(targetLinearVelocityMagnitude, previousHeading);

    // Apply rotation deadband
    double omega =
        (targetLinearVelocityMagnitude > 0)
            ? angleController.calculate(
                currentDriveRotation.getRadians(),
                isFlipped
                    ? linearVelocity.getAngle().getRadians()
                    : linearVelocity.unaryMinus().getAngle().getRadians())
            : 0.0;

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? currentDriveRotation.plus(new Rotation2d(Math.PI)) : currentDriveRotation));
  }
}
