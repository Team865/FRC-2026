package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShopTesting {
  private ShopTesting() {}

  public static void enable(
      CommandXboxController driverController,
      Serializer serializer,
      BallTunneler ballTunneler,
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      DoubleSupplier turretCameraTXGetter) {
    LoggedTunableNumber hoodTestAngle = new LoggedTunableNumber("Test/HoodAngleDeg", 0.0);
    LoggedTunableNumber flywheelTestVelocityRadsPerSec =
        new LoggedTunableNumber("Test/FlywheelVelocityRadsPerSec", 0.0);
    LoggedTunableNumber turretTestAngle = new LoggedTunableNumber("Test/turretAngleDeg", 0.0);
    Angle turretAngleIncrementRate = Degrees.of(90.0).times(0.020);

    driverController
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                serializer.runSerializer(),
                ballTunneler.runVolts(9),
                flywheel.runVelocity(
                    () -> RadiansPerSecond.of(flywheelTestVelocityRadsPerSec.get()))));

    driverController.leftBumper().onTrue(hood.setTargetAngle(Degrees.zero()));

    // driverController.rightTrigger().whileTrue(flywheel.runVolts(() ->
    // flywheelTestVoltage.get()));

    hood.setDefaultCommand(hood.runVoltage(() -> -driverController.getLeftY()));
    turret.setDefaultCommand(
        turret
            .setTargetAngle(turret.getOrientation())
            .andThen(
                turret.lockOntoTarget(
                    new Supplier<Angle>() {
                      Angle currentTargetAngle = turret.getOrientation();

                      @Override
                      public Angle get() {
                        if (driverController.povLeft().getAsBoolean()) {
                          currentTargetAngle = currentTargetAngle.plus(turretAngleIncrementRate);
                        }

                        if (driverController.povRight().getAsBoolean()) {
                          currentTargetAngle = currentTargetAngle.minus(turretAngleIncrementRate);
                        }

                        return currentTargetAngle;
                      }
                    },
                    () -> 0.0)));

    driverController
        .leftTrigger()
        .whileTrue(
            turret.lockOntoTarget(
                () ->
                    turret
                        .getOrientation()
                        .plus(Degrees.of(Math.round(turretCameraTXGetter.getAsDouble()))),
                () -> 0.0));

    // driverController
    //     .y()
    //     .onTrue(turret.lockOntoTarget(() -> Degrees.of(turretTestAngle.get()), () -> 0.0));
  }
}
