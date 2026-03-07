package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  // Gains
  private static final LoggedTunableNumber translationKp =
      new LoggedTunableNumber("DriveToPose/translationKp", 100.0);
  private static final LoggedTunableNumber translationKd =
      new LoggedTunableNumber("DriveToPose/translationKd", 0.0);
  private static final LoggedTunableNumber maxVelocityMPS =
      new LoggedTunableNumber("DriveToPose/MaxVelocityMetersPerSec", 5.0);
  private static final LoggedTunableNumber linAccelMPS2 =
      new LoggedTunableNumber("DriveToPose/AccelerationMetersPerSec2", 5.0);
  private static final LoggedTunableNumber translationToleranceInches =
      new LoggedTunableNumber("DriveToPose/translationToleranceInches", 0.5);

  private static final LoggedTunableNumber rotationKp =
      new LoggedTunableNumber("DriveToPose/rotationKp", 10.0);
  private static final LoggedTunableNumber rotationKd =
      new LoggedTunableNumber("DriveToPose/rotationKd", 0.0);
  private static final LoggedTunableNumber maxOmegaDPS =
      new LoggedTunableNumber("DriveToPose/MaxOmegaDegPerSec", 1.0);
  private static final LoggedTunableNumber angAccelDPS2 =
      new LoggedTunableNumber("DriveToPose/AccelerationDegPerSec", 1.0);
  private static final LoggedTunableNumber rotationToleranceDeg =
      new LoggedTunableNumber("DriveToPose/rotationToleranceDeg", 2);

  private final Drive drive;
  private final Supplier<Pose2d> targetSupplier;

  private final ProfiledPIDController translationController =
      new ProfiledPIDController(0, 0, 0, new Constraints(maxVelocityMPS.get(), linAccelMPS2.get()));
  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          0,
          0,
          0,
          new Constraints(
              Units.degreesToRadians(maxOmegaDPS.get()),
              Units.degreesToRadians(angAccelDPS2.get())));

  private double initialDistanceMeters = 0.0;
  private double distanceToReachMaxVelocityMeters = 0.0;
  private boolean canFullTrapezoidBeConstructed = false;

  public DriveToPose(Supplier<Pose2d> targetSupplier, Drive drive) {
    this.drive = drive;
    this.targetSupplier = targetSupplier;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  private double calculateVelocityMetersPerSec(double currentDistanceMeters) {
    double distanceTraveledMeters = Math.abs(initialDistanceMeters - currentDistanceMeters);

    if (canFullTrapezoidBeConstructed) {
      if (distanceTraveledMeters < distanceToReachMaxVelocityMeters) {
        return Math.sqrt(2 * linAccelMPS2.get() * distanceTraveledMeters);
      } else if (distanceTraveledMeters
          > initialDistanceMeters - distanceToReachMaxVelocityMeters) {
        return Math.sqrt(2 * linAccelMPS2.get() * (initialDistanceMeters - distanceTraveledMeters));
      } else {
        return maxVelocityMPS.get();
      }
    } else {
      if (distanceTraveledMeters <= (initialDistanceMeters / 2)) {
        return Math.sqrt(2 * linAccelMPS2.get() * distanceTraveledMeters);
      } else {
        return Math.sqrt(2 * linAccelMPS2.get() * (initialDistanceMeters - distanceTraveledMeters));
      }
    }
  }

  @Override
  public void initialize() {
    initialDistanceMeters = getDistanceMeters(drive.getPose(), targetSupplier.get());

    double maxVelocity = maxVelocityMPS.get();
    double linAccel = linAccelMPS2.get();
    distanceToReachMaxVelocityMeters = (maxVelocity * maxVelocity) / (2 * linAccel);

    canFullTrapezoidBeConstructed = initialDistanceMeters > (2 * distanceToReachMaxVelocityMeters);
  }

  @Override
  public void execute() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> translationController.setPID(c[0], 0, c[1]), translationKp, translationKd);
    LoggedTunableNumber.ifChanged(
        id, c -> rotationController.setPID(c[0], 0, c[1]), rotationKp, rotationKd);

    LoggedTunableNumber.ifChanged(
        id,
        c -> translationController.setConstraints(new Constraints(c[0], c[1])),
        maxVelocityMPS,
        linAccelMPS2);
    LoggedTunableNumber.ifChanged(
        id,
        c ->
            rotationController.setConstraints(
                new Constraints(Units.degreesToRadians(c[0]), Units.degreesToRadians(c[1]))),
        maxOmegaDPS,
        angAccelDPS2);

    Pose2d drivePose = drive.getPose();
    Pose2d targetPose = targetSupplier.get();
    Rotation2d angleToTarget =
        targetPose.getTranslation().minus(drivePose.getTranslation()).getAngle();

    // Calculate translation velocity
    double speedMetersPerSec =
        calculateVelocityMetersPerSec(getDistanceMeters(drivePose, targetPose));
    // translationController.calculate(
    //     initialDistanceMeters - getDistanceMeters(drivePose, targetPose),
    //     initialDistanceMeters);

    // Calculate omega
    double omegaRadsPerSec =
        rotationController.calculate(
            drivePose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            speedMetersPerSec * angleToTarget.getCos(),
            speedMetersPerSec * angleToTarget.getSin(),
            omegaRadsPerSec);
    ChassisSpeeds targetVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drivePose.getRotation());

    Logger.recordOutput("DriveToPose/FieldRelativeSpeeds", fieldRelativeSpeeds);
    Logger.recordOutput("DriveToPose/ChassisSpeeds", targetVelocity);

    drive.runVelocity(targetVelocity);
  }

  private double getDistanceMeters(Pose2d currentPose, Pose2d targetPose) {
    return currentPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  public Trigger atTranslationSetpoint() {
    return new Trigger(
        () ->
            drive.getPose().getTranslation().getDistance(targetSupplier.get().getTranslation())
                < Units.inchesToMeters(translationToleranceInches.get()));
  }

  public Trigger atRotationSetpoint() {
    return new Trigger(
        () ->
            drive.getRotation().minus(targetSupplier.get().getRotation()).getDegrees()
                < rotationToleranceDeg.get());
  }

  public Trigger atPoseSetpoint() {
    return new Trigger(
        () -> {
          Pose2d drivePose = drive.getPose();
          Pose2d targetPose = targetSupplier.get();

          boolean isAtTranslationSetpoint =
              drivePose.getTranslation().getDistance(targetPose.getTranslation())
                  < Units.inchesToMeters(translationToleranceInches.get());

          boolean isAtRotationSetpoint =
              Math.abs(drivePose.getRotation().minus(targetPose.getRotation()).getDegrees())
                  < rotationToleranceDeg.get();

          return isAtTranslationSetpoint && isAtRotationSetpoint;
        });
  }
}
