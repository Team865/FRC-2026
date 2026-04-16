// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.VisionUtil;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final IntegerPublisher throttlePublisher;
  private final IntegerPublisher imuModePublisher;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private final String name;
  private final boolean isLL4;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   * @param isLL4
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier, boolean isLL4) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.rotationSupplier = rotationSupplier;
    this.isLL4 = isLL4;

    if (isLL4) VisionUtil.registerLimelight4IO(this);

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    throttlePublisher = table.getIntegerTopic("throttle_set").publish();
    imuModePublisher = table.getIntegerTopic("imumode_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    reseed();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 2000;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    Rotation2d yaw = rotationSupplier.get();
    orientationPublisher.accept(new double[] {yaw.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

    if (isLL4) {
      imuModePublisher.accept(4);
    }

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // MegaTag 1
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      if (isLL4) {
        poseObservations.add(
            new PoseObservation(
                // Timestamp, based on server timestamp of publish and latency
                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                // 3D pose estimate
                parsePose(rawSample.value),

                // Ambiguity, using only the first tag because ambiguity isn't applicable for
                // multitag
                rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

                // Tag count
                (int) rawSample.value[7],

                // Average tag distance
                rawSample.value[9],

                // Observation type
                PoseObservationType.MEGATAG_1));
      }
    }
    if (!isLL4) {
      for (var rawSample : megatag2Subscriber.readQueue()) {
        if (rawSample.value.length == 0) continue;
        for (int i = 11; i < rawSample.value.length; i += 7) {
          tagIds.add((int) rawSample.value[i]);
        }
        poseObservations.add(
            new PoseObservation(
                rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                parsePose(rawSample.value),
                0.0,
                (int) rawSample.value[7],
                rawSample.value[9],
                PoseObservationType.MEGATAG_2));
      }
    }
    // Save pose observations to inputs object
    inputs.poseObservations =
        poseObservations.toArray(new PoseObservation[poseObservations.size()]);

    // Save tag IDs to inputs object
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Math.toRadians(rawLLArray[3]),
            Math.toRadians(rawLLArray[4]),
            Math.toRadians(rawLLArray[5])));
  }

  public void reseed() {
    if (!isLL4) return;

    imuModePublisher.accept(1);
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public String getName() {
    return this.name;
  }

  @Override
  public void throttleCamera(int throttleAmount) {
    if (this.isLL4) {
      throttlePublisher.accept(throttleAmount);
    }
  }
}
