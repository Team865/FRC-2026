package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;

public class ShopTesting {
  private ShopTesting() {}

  public static void enable(
      CommandXboxController driverController, Flywheel flywheel, Hood hood, Turret turret) {
    LoggedTunableNumber hoodTestAngle = new LoggedTunableNumber("Test/HoodAngleDeg", 0.0);
    LoggedTunableNumber flywheelTestVoltage = new LoggedTunableNumber("Test/FlywheelVoltage", 0.0);
    LoggedTunableNumber flywheelTestVelocityRadsPerSec =
        new LoggedTunableNumber("Test/FlywheelVelocityRadsPerSec", 0.0);

    driverController.leftBumper().onTrue(hood.setTargetAngle(Degrees.zero()));

    driverController.rightTrigger().whileTrue(flywheel.runVolts(() -> flywheelTestVoltage.get()));
    driverController
        .leftTrigger()
        .whileTrue(
            flywheel.runVelocity(() -> RadiansPerSecond.of(flywheelTestVelocityRadsPerSec.get())));

    hood.setDefaultCommand(hood.runVoltage(() -> -driverController.getLeftY()));
  }
}
