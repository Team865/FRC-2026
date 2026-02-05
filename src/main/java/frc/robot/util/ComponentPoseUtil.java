package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import org.littletonrobotics.junction.Logger;

public class ComponentPoseUtil {

  private ComponentPoseUtil() {}

  public static void publishComponentPoses(
      Rollers rollers, Pivot turretPivot, Pivot intakePivot, Climber climber) {

    Pose3d[] poses = new Pose3d[4];
    // Serializer
    poses[0] =
        rollers != null
            ? new Pose3d(new Translation3d(), new Rotation3d(0, 0, rollers.getPositionRads()))
            : new Pose3d();

    // Turret
    poses[1] =
        turretPivot != null && turretPivot.getOrientation() != null
            ? new Pose3d(
                new Translation3d(), new Rotation3d(0, 0, turretPivot.getOrientation().in(Radians)))
            : new Pose3d();

    // Intake arm
    poses[2] =
        intakePivot != null && intakePivot.getOrientation() != null
            ? new Pose3d(
                new Translation3d(-0.31, 0, 0.21),
                new Rotation3d(0, intakePivot.getOrientation().in(Radians), 0))
            : new Pose3d();

    // Climber
    poses[3] =
        climber != null
            ? new Pose3d(new Translation3d(0, 0, climber.getPositionMeters()), new Rotation3d())
            : new Pose3d();

    Logger.recordOutput("Robot/ComponentPoses", poses);
  }
}
