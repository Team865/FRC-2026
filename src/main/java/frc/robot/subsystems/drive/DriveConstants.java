package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AllianceFlipUtil;

public final class DriveConstants {
  public static final LinearVelocity bumpModeMaxSpeed = TunerConstants.kSpeedAt12Volts.times(0.5);
  public static final LinearVelocity shootingModeMaxSpeed = MetersPerSecond.of(1.0);

  public static Rotation2d getZeroOrientation() {
    return AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero;
  }

  public static final double driveMotorSupplyCurrentLimit = 50.0;

  private DriveConstants() {}
}
