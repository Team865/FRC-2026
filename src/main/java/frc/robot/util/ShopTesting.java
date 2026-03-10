package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShopTesting {
  private ShopTesting() {}

  public static void enable(
      CommandXboxController driverController,
      Drive drive,
      Serializer serializer,
      BallTunneler ballTunneler,
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Intake intake,
      Supplier<Pose2d> hubPoseSupplier,
      DoubleSupplier hubDistanceSupplier) {
    LoggedTunableNumber flywheelTestVelocityRadsPerSec =
        new LoggedTunableNumber("Test/FlywheelVelocityRadsPerSec", 0.0);
    LoggedTunableNumber turretTestAngle = new LoggedTunableNumber("Test/turretAngleDeg", 0.0);
    Angle turretAngleIncrementRate = Degrees.of(90.0).times(0.020);
    LoggedTunableNumber hoodTestAngleDeg = new LoggedTunableNumber("Test/HoodAngleDeg", 0.0);
    LoggedTunableNumber shootOnTheMoveFactor =
        new LoggedTunableNumber("Test/ShootOnTheMoveFactor", 1.0);

    LoggedTunableNumber rollersTestVoltage =
        new LoggedTunableNumber("Test/IntakeRollersVoltage", 0.0);
    LoggedTunableNumber rollersTestVelMPS =
        new LoggedTunableNumber("Test/IntakeRollersVelocityMetersPerSec", 0.0);
    LoggedTunableNumber extensionTestVoltage =
        new LoggedTunableNumber("Test/IntakeExtVoltage", 0.0);
    LoggedTunableNumber extensionTestPositionInches =
        new LoggedTunableNumber("Test/IntakeExtPosInches", 0.0);

    // driverController
    //     .leftTrigger()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             serializer.runSerializer(),
    //             ballTunneler.runTunneler(),
    //             flywheel.runVelocity(
    //                 () -> RadiansPerSecond.of(flywheelTestVelocityRadsPerSec.get()))));

    // driverController
    //     .rightTrigger()
    //     .whileTrue(intake.rollers.runVolts(() -> rollersTestVoltage.get()));
    driverController
        .rightTrigger()
        .whileTrue(
            intake.rollers.runLinearVelocity(() -> MetersPerSecond.of(rollersTestVelMPS.get())));
    // driverController.povUp().whileTrue(intake.stow());
    // driverController.povDown().onTrue(intake.deploy());

    // driverController.leftBumper().onTrue(hood.setTargetAngle(Degrees.zero()));

    // driverController
    //     .rightTrigger()
    //     .whileTrue(ballTunneler.runVolts(() -> ballTunnelerTestVoltage.get()));

    // driverController
    //     .y()
    //     .whileTrue(
    //         turret.lockOntoTarget(
    //             () ->
    //                 ShootingUtil.calculateTurretRelativeAngle(
    //                     drive.getPose(), hubPoseSupplier.get())))
    //     .onTrue(hood.setTargetAngle(() -> Degrees.of(hoodTestAngleDeg.get())));

    // hood.setDefaultCommand(hood.runVoltage(() -> -driverController.getLeftY()));
    // turret.setDefaultCommand(
    //     turret.runVoltage(
    //         new DoubleSupplier() {
    //           private final Trigger left = driverController.povLeft();
    //           private final Trigger right = driverController.povRight();

    //           public double getAsDouble() {
    //             return left.getAsBoolean()
    //                 ? turretTestVoltage.get()
    //                 : right.getAsBoolean() ? -turretTestVoltage.get() : 0.0;
    //           }
    //         }));

    ShootingLogger shootingLogger = new ShootingLogger();
    Subsystem proxySubsystem = new Subsystem() {};

    driverController
        .a()
        .onTrue(
            Commands.runOnce(
                new Runnable() {
                  private int numMeasurements = 0;

                  @Override
                  public void run() {
                    this.numMeasurements++;
                    shootingLogger.addMeasurement(
                        hubDistanceSupplier.getAsDouble(),
                        flywheelTestVelocityRadsPerSec.get(),
                        Units.degreesToRadians(hoodTestAngleDeg.get()));
                    Logger.recordOutput("Number Of Shooting Measurements Taken", numMeasurements);
                  }
                }));

    SimpleDateFormat dateFormatter = new SimpleDateFormat("MM-dd-yyyy_HH-mm-ss");

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                () ->
                    shootingLogger.writeToFile(
                        String.format(
                            "/U/logs/measurements_%s.txt", dateFormatter.format(new Date()))),
                proxySubsystem));

    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         turret.lockOntoTarget(
    //             () ->
    //                 ShootingUtil.calculateTurretRelativeAngle(
    //                     drive.getPose(),
    //                     ShootingUtil.correctTargetPoseWhileMoving(
    //                         hubPoseSupplier.get(),
    //                         drive.getFieldOrientedSpeeds(),
    //                         shootOnTheMoveFactor.get()))))
    //     .whileTrue(
    //         hood.runTargetAngle(
    //             () -> {
    //               Angle targetAngle =
    //                   ShootingUtil.calculateHoodAngle(
    //                       drive.getPose(),
    //                       ShootingUtil.correctTargetPoseWhileMoving(
    //                           hubPoseSupplier.get(),
    //                           drive.getFieldOrientedSpeeds(),
    //                           shootOnTheMoveFactor.get()));
    //               Logger.recordOutput("Calculated Hood Angle", targetAngle.in(Degrees));
    //               return targetAngle;
    //             }));

    // driverController
    //     .x()
    //     .whileTrue(
    //         turret.lockOntoTarget(
    //             () ->
    //                 ShootingUtil.calculateTurretRelativeAngle(
    //                     drive.getPose(), hubPoseSupplier.get()),
    //             () -> RadiansPerSecond.of(drive.getAngularVelocityRadPerSec())));

    // driverController
    //     .y()
    //     .onTrue(turret.lockOntoTarget(() -> Degrees.of(turretTestAngle.get()), () -> 0.0));
  }
}
