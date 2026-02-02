// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim implements VisionIO {
  private static VisionSystemSim visionSim;
  private final String name;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera Transform from robot to camera.
   * @param poseSupplier Supplier for the robot pose in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this.name = name;
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties =
        cameraProperties.setCalibration(
            640,
            480,
            Rotation2d.fromDegrees(
                91.2)); // limelight4 FOV, calculated from horizontal and vertical
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());

    inputs.connected = camera.isConnected();

    Set<Short> seenTagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {

      if (result.hasTargets()) {
        var best = result.getBestTarget();
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(best.getYaw()), Rotation2d.fromDegrees(best.getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      if (result.multitagResult.isPresent()) {
        var multitag = result.multitagResult.get();
        Transform3d fieldToCamera = multitag.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        double totalDistance = 0.0;
        for (var tgt : result.targets) {
          totalDistance += tgt.bestCameraToTarget.getTranslation().getNorm();
        }

        seenTagIds.addAll(multitag.fiducialIDsUsed);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                multitag.estimatedPose.ambiguity,
                multitag.fiducialIDsUsed.size(),
                totalDistance / Math.max(1, result.targets.size()),
                PoseObservationType.PHOTONVISION));
      } else if (!result.targets.isEmpty()) {
        var target = result.targets.get(0);
        var tagPoseOpt = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPoseOpt.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPoseOpt.get().getTranslation(), tagPoseOpt.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          seenTagIds.add((short) target.fiducialId);

          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  cameraToTarget.getTranslation().getNorm(),
                  PoseObservationType.PHOTONVISION));
        }
      }
    }

    inputs.poseObservations =
        poseObservations.toArray(new PoseObservation[poseObservations.size()]);

    inputs.tagIds = new int[seenTagIds.size()];
    int idx = 0;
    for (int id : seenTagIds) {
      inputs.tagIds[idx++] = id;
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
