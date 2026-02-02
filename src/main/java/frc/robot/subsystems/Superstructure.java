package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Turret;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static enum SuperState {
    IDLE,
    READYING_SHOOTER,
    READY_TO_SHOOT,
    EXTENDING_CLIMBER,
    RETRACTING_CLIMBER,
    CLIMBER_DEPLOYED
  }

  public static enum RequestCode {
    /** State change request succeeded */
    SUCCESS,
    /**
     * This state doesn't have requirements, meaning it can't be requested. This likely means it is
     * a state used internally, rather a state that is meant to be requested externally.
     */
    CANNOT_REQUEST,
    /** The requirements to switch to the state aren't met */
    REQUIREMENTS_FAILED,
    /** The current state is already set to this */
    ALREADY_SET
  }

  private SuperState currentState = SuperState.IDLE;

  private Map<SuperState, Trigger> stateTriggers =
      new HashMap<>(); // Triggers for when a state is entered or exited
  private Map<SuperState, BooleanSupplier> stateRequirements =
      new HashMap<>(); // BooleanSuppliers which return whether a state can be switched to

  private final Drive drive;
  private final Turret turret;
  private final Supplier<Pose2d> hubPoseSupplier;

  public Superstructure(Drive drive, Turret turret, Supplier<Pose2d> hubPoseSupplier) {
    this.drive = drive;
    this.turret = turret;
    this.hubPoseSupplier = hubPoseSupplier;

    // Initialize state triggers
    for (SuperState state : SuperState.values()) {
      stateTriggers.put(
          state,
          new Trigger(
              () ->
                  this.currentState.equals(state) // Is the current state equal to this state
                      && DriverStation.isEnabled() // Is the robot enabled
                      && DriverStation.isTeleop() // Is the robot in teleop
              ));
    }

    configureStateRequirements();
    configureStateBehaviours();
  }

  /** Configure the requirements of each state */
  public void configureStateRequirements() {
    // Can always be set to idle
    stateRequirements.put(SuperState.IDLE, () -> true);
  }

  /** Configure what the behaviours of each state are */
  public void configureStateBehaviours() {
    // stateTriggers.get(SuperState.IDLE).onTrue(Command); Example Command
  }

  public SuperState getCurrentState() {
    return this.currentState;
  }

  /**
   * Attempt to switch state to a requested state. Returns a request code informing whether it
   * succeeded.
   *
   * @param targetState The state to switch to
   * @return Whether the request was a success or not.
   */
  public RequestCode requestState(SuperState targetState) {
    // If the state is already set
    if (currentState.equals(targetState)) return RequestCode.ALREADY_SET;
    // If no state requirement exists
    if (!stateRequirements.containsKey(targetState)) return RequestCode.CANNOT_REQUEST;
    // If the state requirement isn't met
    if (!stateRequirements.get(targetState).getAsBoolean()) return RequestCode.REQUIREMENTS_FAILED;

    forceState(targetState);
    return RequestCode.SUCCESS;
  }

  /**
   * Forcibly sets the current state to the specified state. Should only be used for resetting to
   * IDLE or demo-ing
   *
   * @param targetState The state to switch to
   */
  public void forceState(SuperState targetState) {
    this.currentState = targetState;
  }

  /**
   * A command that continuously runs <code>requestState</code> until interrupted
   *
   * @see #requestState
   */
  public Command requestStateCommand(SuperState targetState) {
    return runOnce(() -> requestState(targetState));
  }

  /**
   * A command that achieves the same thing as <code>forceState</code>
   *
   * @see #forceState
   */
  public Command forceStateCommand(SuperState targetState) {
    return runOnce(() -> forceState(targetState));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Superstructure/Components/TurretTarget",
        new Pose2d(
            drive
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(hubPoseSupplier.get().getTranslation()),
                        drive.getRotation().plus(new Rotation2d(turret.getOrientation())))),
            Rotation2d.kZero));
  }
}
