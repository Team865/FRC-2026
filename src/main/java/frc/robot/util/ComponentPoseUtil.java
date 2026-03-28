package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import org.littletonrobotics.junction.Logger;

public class ComponentPoseUtil {
  private static final double hoodPitchOffsetRads = Units.degreesToRadians(-26.5);
  private static final double hoodHorizontalOffsetMeters = 0.10180629334;

  private static final double lintakeVFactor = -Math.sin(Units.degreesToRadians(17.5));
  private static final double lintakeHFactor = -Math.cos(Units.degreesToRadians(17.5));

  private ComponentPoseUtil() {}

  public static void publishComponentPoses(
      Serializer serializer,
      Turret turretPivot,
      Hood hood,
      Extension intakeExtension,
      Climber climber) {

    Pose3d[] poses = new Pose3d[6];
    // Serializer
    poses[0] =
        new Pose3d(
            Translation3d.kZero, new Rotation3d(0, 0, -serializer.getPosition().in(Radians)));

    // Turret
    double turretOrientationRads = turretPivot.getOrientation().in(Radians);

    poses[1] = new Pose3d(Translation3d.kZero, new Rotation3d(0, 0, turretOrientationRads));

    // Hood
    poses[2] =
        new Pose3d(
            new Translation3d(
                Math.cos(turretOrientationRads) * hoodHorizontalOffsetMeters,
                Math.sin(turretOrientationRads) * hoodHorizontalOffsetMeters,
                0.635),
            new Rotation3d(
                0, hoodPitchOffsetRads + hood.getOrientation().in(Radians), turretOrientationRads));

    // Intake arm
    double intakeExtensionMeters = intakeExtension.getPosition().in(Meters);
    double intakeExtensionHorizontal = lintakeHFactor * intakeExtensionMeters;

    poses[3] =
        new Pose3d(
            new Translation3d(intakeExtensionHorizontal, 0, lintakeVFactor * intakeExtensionMeters),
            Rotation3d.kZero);

    poses[4] = new Pose3d(new Translation3d(intakeExtensionHorizontal, 0, 0), Rotation3d.kZero);

    // Climber
    poses[5] = new Pose3d(new Translation3d(0, 0, climber.getPositionMeters()), Rotation3d.kZero);

    Logger.recordOutput("RobotRendering/ComponentPoses", poses);
  }
}
