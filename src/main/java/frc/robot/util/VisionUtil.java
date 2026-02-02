package frc.robot.util;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import java.util.LinkedList;
import java.util.List;

public final class VisionUtil {

  private VisionUtil() {}

  public static Translation3d[] makeTargetLines(VisionIOInputsAutoLogged inputs) {
    Translation3d[] lines = new Translation3d[inputs.tagIds.length * 2];
    int idx = 0;
    Pose3d robotPose3d =
        inputs.poseObservations.length > 0 ? inputs.poseObservations[0].pose() : new Pose3d();

    for (int tagId : inputs.tagIds) {
      var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
      if (tagPoseOpt.isPresent()) {
        Pose3d tagPose3d = tagPoseOpt.get();
        lines[idx++] = robotPose3d.getTranslation();
        lines[idx++] = tagPose3d.getTranslation();
      }
    }

    return lines;
  }

  public static PoseProcessingResult processPoseObservations(
      VisionIOInputsAutoLogged inputs, VisionConsumer consumer, int cameraIndex) {
    List<Pose3d> robotPoses = new LinkedList<>();
    List<Pose3d> robotPosesAccepted = new LinkedList<>();
    List<Pose3d> robotPosesRejected = new LinkedList<>();

    for (var observation : inputs.poseObservations) {
      boolean rejectPose =
          observation.tagCount() == 0
              || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
              || Math.abs(observation.pose().getZ()) > maxZError
              || observation.pose().getX() < 0.0
              || observation.pose().getX() > aprilTagLayout.getFieldLength()
              || observation.pose().getY() < 0.0
              || observation.pose().getY() > aprilTagLayout.getFieldWidth();

      robotPoses.add(observation.pose());
      if (rejectPose) {
        robotPosesRejected.add(observation.pose());
        continue;
      } else {
        robotPosesAccepted.add(observation.pose());
      }

      double stdDevFactor =
          Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      double linearStdDev = linearStdDevBaseline * stdDevFactor;
      double angularStdDev = angularStdDevBaseline * stdDevFactor;

      if (observation.type() == VisionIO.PoseObservationType.MEGATAG_2) {
        linearStdDev *= linearStdDevMegatag2Factor;
        angularStdDev *= angularStdDevMegatag2Factor;
      }

      if (cameraIndex < cameraStdDevFactors.length) {
        linearStdDev *= cameraStdDevFactors[cameraIndex];
        angularStdDev *= cameraStdDevFactors[cameraIndex];
      }

      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    return new PoseProcessingResult(robotPoses, robotPosesAccepted, robotPosesRejected);
  }

  public static class PoseProcessingResult {
    public final List<Pose3d> all;
    public final List<Pose3d> accepted;
    public final List<Pose3d> rejected;

    public PoseProcessingResult(List<Pose3d> all, List<Pose3d> accepted, List<Pose3d> rejected) {
      this.all = all;
      this.accepted = accepted;
      this.rejected = rejected;
    }
  }
}
