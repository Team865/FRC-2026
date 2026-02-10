package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.VisionUtil;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Map<String, VisionConsumer> cameraConsumers;
  private final Map<String, Integer> cameraNameToIndex = new HashMap<>();

  private double turretTxDegrees = 0.0;
  private int turretSeenTagId = -1;
  private int turretCandidateTagId = -1;
  private int turretTagFrames = 0;
  private static final int TURRET_DEBOUNCE_FRAMES = 3;

  public Vision(Drive drive, VisionIO... io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlerts = new Alert[io.length];
    this.cameraConsumers = new HashMap<>();

    // Initalize the camera IOs
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      cameraNameToIndex.put(io[i].getName(), i);
      disconnectedAlerts[i] =
          new Alert("Vision camera " + io[i].getName() + " is disconnected.", AlertType.kWarning);
    }

    // Drivetrain camera consumer, outputs to the drivetrain pose
    VisionConsumer drivetrainConsumer =
        (pose, ts, stdDevs) -> {
          drive.addVisionMeasurement(pose, ts, stdDevs);
        };

    cameraConsumers.put(camera0Name, drivetrainConsumer);
    cameraConsumers.put(camera1Name, drivetrainConsumer);

    // Turret Camera consumer, outputs the horizontal offset from the current alliances hub tag
    // debounced
    cameraConsumers.put(
        camera2Name,
        (pose, ts, stdDevs) -> {
          int cameraIndex = cameraNameToIndex.get(camera2Name);
          VisionIOInputsAutoLogged inputs2 = inputs[cameraIndex];
          int[] seenTags = inputs2.tagIds;

          double tx = 0.0;
          int newCandidate = -1;
          for (int hubTagId : AllianceFlipUtil.apply(FieldConstants.hubTagIds)) {
            if (IntStream.of(seenTags).anyMatch(t -> t == hubTagId)) {
              newCandidate = hubTagId;
              if (inputs2.latestTargetObservation != null) {
                tx = inputs2.latestTargetObservation.tx().getDegrees();
              }
              break;
            }
          }
          if (newCandidate == turretCandidateTagId) {
            turretTagFrames++;
          } else {
            turretCandidateTagId = newCandidate;
            turretTagFrames = 1;
          }
          if (turretTagFrames >= TURRET_DEBOUNCE_FRAMES) {
            turretSeenTagId = turretCandidateTagId;
            turretTxDegrees = tx;
          }
        });
  }

  public VisionIOInputsAutoLogged getInputs(int cameraIndex) {
    return inputs[cameraIndex];
  }

  public int getCameraIndex(String cameraName) {
    Integer idx = cameraNameToIndex.get(cameraName);
    if (idx == null) {
      throw new IllegalArgumentException("Camera name not found: " + cameraName);
    }
    return idx;
  }

  public double getTurretTxDegrees() {
    return turretTxDegrees;
  }

  public int getTurretSeenTagId() {
    return turretSeenTagId;
  }

  @Override
  public void periodic() {
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      String cameraName = io[i].getName();
      Logger.processInputs("Vision/" + cameraName, inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);

      List<Pose3d> tagPoses = new LinkedList<>();
      for (int tagId : inputs[i].tagIds) {
        VisionConstants.aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      VisionConsumer consumer = cameraConsumers.get(cameraName);
      VisionUtil.PoseProcessingResult poseResult =
          VisionUtil.processPoseObservations(inputs[i], consumer, i);

      Logger.recordOutput("Vision/" + cameraName + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraName + "/RobotPoses", poseResult.all.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraName + "/RobotPosesAccepted",
          poseResult.accepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraName + "/RobotPosesRejected",
          poseResult.rejected.toArray(new Pose3d[0]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(poseResult.all);
      allRobotPosesAccepted.addAll(poseResult.accepted);
      allRobotPosesRejected.addAll(poseResult.rejected);
    }

    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public static Vision createPerCameraVision(Drive drive, VisionIO... io) {
    return new Vision(drive, io);
  }
}
