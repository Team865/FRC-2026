package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.VisionUtil;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Map<String, VisionConsumer> cameraConsumers;
  private final Map<String, Integer> cameraNameToIndex = new HashMap<>();
  private final Debouncer shouldUseTurretLLDebouncer = new Debouncer(1.0, DebounceType.kRising);

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
    VisionConsumer positionConsumer =
        (pose, ts, stdDevs) -> {
          drive.addVisionMeasurement(pose, ts, stdDevs);
        };

    VisionConsumer turretConsumer =
        (pose, ts, stdDevs) -> {
          if (shouldUseTurretLLDebouncer.calculate(
              inputs[0].tagIds.length + inputs[1].tagIds.length == 0)) {
            pose = pose.plus(new Transform2d(-turretForwardOffsetMeters, 0, Rotation2d.kZero));
            drive.addVisionMeasurement(pose, ts, stdDevs);
          }
        };

    cameraConsumers.put(camera0Name, positionConsumer);
    cameraConsumers.put(camera1Name, positionConsumer);
    cameraConsumers.put(camera2Name, turretConsumer);
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

  @Override
  public void periodic() {
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
      if (consumer == null) continue;
      VisionUtil.PoseProcessingResult poseResult =
          VisionUtil.processPoseObservations(inputs[i], consumer, i);

      Logger.recordOutput(
          "Vision/" + cameraName + "/RobotPosesAccepted",
          poseResult.accepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + cameraName + "/RobotPosesRejected",
          poseResult.rejected.toArray(new Pose3d[0]));
      allRobotPosesAccepted.addAll(poseResult.accepted);
      allRobotPosesRejected.addAll(poseResult.rejected);
    }
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

  public void throttleCameras(int throttleAmount) {
    for (VisionIO visionIO : io) {
      visionIO.throttleCamera(throttleAmount);
    }
  }
}
