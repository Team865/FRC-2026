package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.ShootingUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  /** The state in regards to shooting */
  public static enum ShootingState {
    /** Requestable: Completely idle */
    IDLE,
    /** Requestable: Start running flywheels and tracking target */
    READYING_SHOOTER,
    /** Intermediate & Requestable: Flywheels are at correct speed, still tracking target */
    READY_TO_SHOOT,
    /** Requestable: Run ball tunneler to start shooting, still tracking target */
    SHOOTING
  }

  /** The state in regards to intaking */
  public static enum IntakingState {
    /** Intermediate: The intake is stowed */
    STOWED,
    /** Requestable: The intake is currently being stowed */
    STOWING,
    /** Requestable: The intake is currently beind deployed */
    DEPLOYING,
    /** Intermediate & Requestable: The intake is deployed and ready to start intaking */
    INTAKE_READY,
    /** Requestable: The intake is currently running */
    INTAKING
  }

  public final StateMachine<ShootingState> shootingStateMachine =
      new StateMachine<>(ShootingState.IDLE);
  public final StateMachine<IntakingState> intakingStateMachine =
      new StateMachine<>(IntakingState.STOWED);

  private final Drive drive;
  private final Intake intake;
  private final Serializer serializer;
  private final BallTunneler ballTunneler;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Supplier<Pose2d> hubPoseSupplier;

  private Pose2d shootingTarget = Pose2d.kZero;
  private double distanceFromShootingTarget = 0.0;

  public Superstructure(
      Drive drive,
      Intake intake,
      Serializer serializer,
      BallTunneler ballTunneler,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Supplier<Pose2d> hubPoseSupplier) {
    this.drive = drive;
    this.intake = intake;
    this.serializer = serializer;
    this.ballTunneler = ballTunneler;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.hubPoseSupplier = hubPoseSupplier;

    // configureStateRequirements();
    // configureStateBehaviours();
    // configureGameStateTriggers();
  }

  /** Configure the requirements of each state */
  private void configureStateRequirements() {
    // Shooting
    shootingStateMachine.stateRequirements.put(
        ShootingState.IDLE, StateMachine.STATE_ALWAYS_AVAILABLE);
    shootingStateMachine.stateRequirements.put(
        ShootingState.READYING_SHOOTER, () -> shootingStateMachine.isInState(ShootingState.IDLE));
    shootingStateMachine.stateRequirements.put(
        ShootingState.READY_TO_SHOOT, () -> shootingStateMachine.isInState(ShootingState.SHOOTING));
    shootingStateMachine.stateRequirements.put(
        ShootingState.SHOOTING, () -> shootingStateMachine.isInState(ShootingState.READY_TO_SHOOT));

    // Intaking
    intakingStateMachine.stateRequirements.put(
        IntakingState.STOWING, StateMachine.STATE_ALWAYS_AVAILABLE);
    intakingStateMachine.stateRequirements.put(
        IntakingState.DEPLOYING, () -> !intakingStateMachine.isInState(IntakingState.INTAKING));
    intakingStateMachine.stateRequirements.put(
        IntakingState.INTAKE_READY, () -> intakingStateMachine.isInState(IntakingState.INTAKING));
    intakingStateMachine.stateRequirements.put(
        IntakingState.INTAKING, () -> intakingStateMachine.isInState(IntakingState.INTAKE_READY));
  }

  /** Configure what the behaviours of each state are */
  private void configureStateBehaviours() {
    // Stop the flywheels when in Idle shooting state
    shootingStateMachine.stateTriggers.get(ShootingState.IDLE).onTrue(flywheel.stop());

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.IDLE)
        .whileFalse( // When the shooting state isn't idle,
            turret.lockOntoTarget( // Have the turret track the target
                () -> ShootingUtil.calculateTurretRelativeAngle(drive.getPose(), shootingTarget),
                () -> RadiansPerSecond.of(drive.getAngularVelocityRadPerSec())))
        .whileFalse( // Have the hood track the target
            hood.trackTarget(() -> ShootingUtil.calculateHoodAngle(distanceFromShootingTarget)))
        .whileFalse(
            flywheel.runVelocity(
                () ->
                    ShootingUtil.getFlywheelVelocity(
                        distanceFromShootingTarget))) // Spin up the flywheels
        .onTrue(runOnce(() -> drive.setMaxLinearSpeed(TunerConstants.kSpeedAt12Volts)))
        .onFalse(runOnce(() -> drive.setMaxLinearSpeed(DriveConstants.shootingModeMaxSpeed)));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.READYING_SHOOTER) // When the flywheels are getting spun up
        .and(flywheel.atTargetVelocity()) // And they are at their target velocity
        .onTrue(forceState(ShootingState.READY_TO_SHOOT)); // then we are ready to shoot

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .whileTrue(ballTunneler.runTunneler());
    // .whileTrue(serializer.runSerializer());

    // Intaking state
    intakingStateMachine
        .stateTriggers
        .get(IntakingState.STOWING)
        .onTrue(intake.stow()) // Stow the intake
        .and(intake.extensionAtSetpoint()) // If the intake is stowed,
        .onTrue(forceState(IntakingState.STOWED)); // move to appropriate state

    intakingStateMachine.stateTriggers.get(IntakingState.DEPLOYING).onTrue(intake.deploy());

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.DEPLOYING) // Deploy the intake
        .and(intake.extensionAtSetpoint()) // If the intake arm is deployed,
        .onTrue(forceState(IntakingState.INTAKE_READY)); // Move to appropriate state

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.INTAKING)
        .whileTrue( // Run the intake based on drivetrain speed
            intake.runRollers(() -> drive.getChassisSpeeds()));

    // shootingStateMachine
    //     .stateTriggers
    //     .get(ShootingState.IDLE)
    //     .and(intakingStateMachine.stateTriggers.get(IntakingState.INTAKING).negate())
    //     // While we are running the shooter in anyway (not Idle), or while we are running the
    // intake
    //     // rollers
    //     .whileFalse(serializer.runSerializer());
  }

  private void configureGameStateTriggers() {
    new Trigger(() -> DriverStation.isTeleopEnabled())
        .onTrue(forceState(ShootingState.IDLE))
        .onTrue(forceState(IntakingState.DEPLOYING));
  }

  /** A command that requests a state for the shooting state machine */
  public Command requestState(ShootingState targetState) {
    return shootingStateMachine.requestStateCommand(targetState);
  }

  /** A command that requests a state for the intaking state machine */
  public Command requestState(IntakingState targetState) {
    return intakingStateMachine.requestStateCommand(targetState);
  }

  /** Continuous requests a state until it is set or this command is interrupted */
  public Command continuouslyRequestState(ShootingState targetState) {
    return shootingStateMachine.runRequestStateCommand(targetState);
  }

  /** Continuous requests a state until it is set or this command is interrupted */
  public Command continuouslyRequestState(IntakingState targetState) {
    return intakingStateMachine.runRequestStateCommand(targetState);
  }

  /** A command that forces a state for the shooting state machine */
  public Command forceState(ShootingState targetState) {
    return shootingStateMachine.forceStateCommand(targetState);
  }

  /** A command that forces a state for the intaking state machine */
  public Command forceState(IntakingState targetState) {
    return intakingStateMachine.forceStateCommand(targetState);
  }

  /** Toggles Shooting Mode */
  public Command toggleShootingMode() {
    return Commands.runOnce(
        () -> {
          if (shootingStateMachine.isInState(ShootingState.IDLE)) {
            shootingStateMachine.requestState(ShootingState.READYING_SHOOTER);
          } else {
            shootingStateMachine.requestState(ShootingState.IDLE);
          }
        },
        shootingStateMachine);
  }

  public Command toggleBumpMode() {
    return runOnce(
        () -> {
          if (drive.getMaxLinearSpeed().equals(DriveConstants.bumpModeMaxSpeed)) {
            if (shootingStateMachine.isInState(ShootingState.IDLE)) {
              drive.setMaxLinearSpeed(TunerConstants.kSpeedAt12Volts);
            } else {
              drive.setMaxLinearSpeed(DriveConstants.shootingModeMaxSpeed);
            }
          } else {
            drive.setMaxLinearSpeed(DriveConstants.bumpModeMaxSpeed);
          }
        });
  }

  public Command toggleIntakeArm() {
    return Commands.runOnce(
        () -> {
          if (intakingStateMachine.isInState(IntakingState.STOWING)
              || intakingStateMachine.isInState(IntakingState.STOWED)) {
            intakingStateMachine.requestState(IntakingState.DEPLOYING);
          } else {
            intakingStateMachine.requestState(IntakingState.STOWING);
          }
        },
        intakingStateMachine);
  }

  @Override
  public void periodic() {
    if (!shootingStateMachine.isInState(ShootingState.IDLE)) {
      shootingTarget =
          ShootingUtil.correctTargetPoseWhileMoving(
              hubPoseSupplier.get(), drive.getFieldOrientedSpeeds());

      distanceFromShootingTarget =
          shootingTarget.getTranslation().getDistance(drive.getPose().getTranslation());
    }

    Logger.recordOutput("Superstructure/ShootingState", shootingStateMachine.getState().toString());
    Logger.recordOutput("Superstructure/IntakingState", intakingStateMachine.getState().toString());
  }
}
