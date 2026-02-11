package frc.robot.util;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.LinkedList;
import java.util.List;

public final class VisionUtil {
  private VisionUtil() {}

  private static final List<VisionIOLimelight> limelight4s = new LinkedList<>();

  public static void reseedAllLimelight4s() {
    for (VisionIOLimelight io : limelight4s) io.reseed();
  }

  public static void registerLimelight4IO(VisionIOLimelight io) {
    limelight4s.add(io);
  }

  public static PoseProcessingResult processPoseObservations(
      VisionIOInputsAutoLogged inputs, VisionConsumer consumer, int cameraIndex) {

    List<Pose3d> allObservedPoses = new LinkedList<>();
    List<Pose3d> acceptedPoses = new LinkedList<>();
    List<Pose3d> rejectedPoses = new LinkedList<>();

    if (inputs.poseObservations.length == 0)
      return new PoseProcessingResult(allObservedPoses, acceptedPoses, rejectedPoses);

    for (var obs : inputs.poseObservations) {
      Pose3d pose = obs.pose();
      allObservedPoses.add(pose);

      boolean isMT2 = obs.type() == VisionIO.PoseObservationType.MEGATAG_2;

      // Accept only MT2 or sim poses
      boolean acceptPose = isMT2 || obs.type() == VisionIO.PoseObservationType.PHOTONVISION;

      if (acceptPose && isMT2) {
        acceptPose =
            obs.tagCount() > 0
                && Math.abs(pose.getZ()) <= maxZError
                && pose.getX() >= 0.0
                && pose.getX() <= aprilTagLayout.getFieldLength()
                && pose.getY() >= 0.0
                && pose.getY() <= aprilTagLayout.getFieldWidth();
      }

      if (!acceptPose) {
        rejectedPoses.add(pose);
        continue;
      }

      acceptedPoses.add(pose);

      double stdDevFactor = Math.pow(obs.averageTagDistance(), 2.0) / obs.tagCount();
      double linearStdDev = linearStdDevBaseline * stdDevFactor;
      double angularStdDev = angularStdDevBaseline * stdDevFactor;

      if (isMT2) {
        linearStdDev *= linearStdDevMegatag2Factor;
        angularStdDev *= angularStdDevMegatag2Factor;
      }

      if (cameraIndex < cameraStdDevFactors.length) {
        linearStdDev *= cameraStdDevFactors[cameraIndex];
        angularStdDev *= cameraStdDevFactors[cameraIndex];
      }

      consumer.accept(
          pose.toPose2d(),
          obs.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    return new PoseProcessingResult(allObservedPoses, acceptedPoses, rejectedPoses);
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
