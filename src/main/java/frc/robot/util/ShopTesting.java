package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.Shooting.ShootingLogger;
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
    LoggedTunableNumber hoodTestAngleDeg = new LoggedTunableNumber("Test/HoodAngleDeg", 0.0);
    LoggedTunableNumber shootOnTheMoveFactor =
        new LoggedTunableNumber("Test/ShootOnTheMoveFactor", 1.0);

    driverController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                serializer.runSerializer(),
                ballTunneler.runTunneler(),
                hood.setTargetAngle(() -> Degrees.of(hoodTestAngleDeg.get())),
                flywheel.runVelocity(() -> RadiansPerSecond.of(450.0))));

    // driverController
    //     .y()
    //     .whileTrue(
    //         turret.lockOntoTarget(
    //             () ->
    //                 ShootingUtil.calculateTurretRelativeAngle(
    //                     drive.getPose(), hubPoseSupplier.get())))
    //     .whileTrue(hood.setTargetAngle(() -> Degrees.of(hoodTestAngleDeg.get())));

    // Turret tracking only when this is held down. Used for velocity compensation testing

    ShootingLogger shootingLogger = new ShootingLogger();
    Subsystem proxySubsystem = new Subsystem() {};

    driverController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                new Runnable() {
                  private int numMeasurements = 0;

                  @Override
                  public void run() {
                    this.numMeasurements++;
                    shootingLogger.addMeasurement(
                        hubDistanceSupplier.getAsDouble(),
                        Units.degreesToRadians(hoodTestAngleDeg.get()),
                        flywheelTestVelocityRadsPerSec.get());
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
  }
}
