package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ComponentPoseUtil {

  /**
   * Publishes component poses to AdvantageScope.
   *
   * @param rollers
   * @param pivots
   */
  public static void publishComponentPoses(List<Rollers> rollers, List<Pivot> pivots) {
    int total = rollers.size() + pivots.size();
    Pose3d[] poses = new Pose3d[total];

    for (int i = 0; i < rollers.size(); i++) {
      Rollers r = rollers.get(i);
      if (r != null) {
        poses[i] =
            new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, r.getPositionRads()));
      } else {

        poses[i] = new Pose3d();
      }
    }

    for (int i = 0; i < pivots.size(); i++) {
      Pivot p = pivots.get(i);
      if (p != null && p.getOrientation() != null) {
        poses[rollers.size() + i] =
            new Pose3d(
                new Translation3d(0, 0, 0), new Rotation3d(0, 0, p.getOrientation().in(Radians)));
      } else {

        poses[rollers.size() + i] = new Pose3d();
      }
    }

    Logger.recordOutput("Robot/ComponentPoses", poses);
  }
}
