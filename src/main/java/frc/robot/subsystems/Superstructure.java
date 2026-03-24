package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
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
    /** Requestable: Rollers, Flywheels, Indexer running */
    SHOOTING,
  }

  /** The state in regards to intaking */
  public static enum IntakingState {
    /** Intermediate: The intake is stowed */
    STOWED,
    /** Requestable: The intake is currently being stowed */
    STOWING,
    /** Requestable: The intake is currently being deployed */
    DEPLOYING,
    /** Intermediate: The intake is deployed and running */
    INTAKING
  }

  public final StateMachine<ShootingState> shootingStateMachine =
      new StateMachine<>(ShootingState.IDLE);
  public final StateMachine<IntakingState> intakingStateMachine =
      new StateMachine<>(IntakingState.STOWED);
  private boolean isManualOverride = false;

  private final Drive drive;
  private final Intake intake;
  private final Serializer serializer;
  private final BallTunneler ballTunneler;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final LEDs leds;
  private final Supplier<Pose2d> hubPoseSupplier;

  private Pose2d shootingTarget = Pose2d.kZero;
  private double distanceFromTargetMeters = 0.0;
  private boolean isPassing = false;

  private final CommandXboxController operatorController;
  private final Trigger manualOverrideTrigger = new Trigger(() -> isManualOverride);
  private final Trigger passingModeTrigger =
      new Trigger(() -> isPassing && DriverStation.isTeleopEnabled() && !isManualOverride);

  public Superstructure(
      Drive drive,
      Intake intake,
      Serializer serializer,
      BallTunneler ballTunneler,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      LEDs leds,
      CommandXboxController operatorController,
      Supplier<Pose2d> hubPoseSupplier) {
    this.drive = drive;
    this.intake = intake;
    this.serializer = serializer;
    this.ballTunneler = ballTunneler;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.leds = leds;
    this.hubPoseSupplier = hubPoseSupplier;
    this.operatorController = operatorController;

    configureStateRequirements();
    configureStateBehaviours();
    configureGameStateTriggers();
    configureShooter();
  }

  /** Configure the requirements of each state */
  private void configureStateRequirements() {
    // Shooting
    shootingStateMachine.stateRequirements.put(
        ShootingState.IDLE, StateMachine.STATE_ALWAYS_AVAILABLE);

    shootingStateMachine.stateRequirements.put(
        ShootingState.SHOOTING, StateMachine.STATE_ALWAYS_AVAILABLE);

    // Intaking
    intakingStateMachine.stateRequirements.put(
        IntakingState.STOWING, StateMachine.STATE_ALWAYS_AVAILABLE);
    intakingStateMachine.stateRequirements.put(
        IntakingState.DEPLOYING, StateMachine.STATE_ALWAYS_AVAILABLE);
    intakingStateMachine.stateRequirements.put(
        IntakingState.INTAKING, StateMachine.STATE_ALWAYS_AVAILABLE);
  }

  /** Configure what the behaviours of each state are */
  private void configureStateBehaviours() {
    // Stop the flywheels when in Idle shooting state
    shootingStateMachine.stateTriggers.get(ShootingState.IDLE).onTrue(leds.idleWaveCommand());
    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(passingModeTrigger.negate())
        .whileTrue(
            flywheel.runVelocity(() -> ShootingUtil.getFlywheelVelocity(distanceFromTargetMeters)))
        .onTrue(leds.shootingWaveCommand());

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(passingModeTrigger)
        .whileTrue(flywheel.runVelocity(() -> RadiansPerSecond.of(350)));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(new Trigger(() -> DriverStation.isTeleopEnabled()))
        .onFalse(drive.setMaxLinearSpeedCmd(TunerConstants.kSpeedAt12Volts))
        .onTrue(drive.setMaxLinearSpeedCmd(DriveConstants.shootingModeMaxSpeed));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(turret.canShoot())
        .whileTrue(ballTunneler.runTunneler())
        .onTrue(serializer.startSerializer())
        .onFalse(serializer.stop());

    // shouldStopSerializer().onTrue(restartSerializerAntiStalled());

    // shootingStateMachine
    //     .stateTriggers
    //     .get(ShootingState.SHOOTING)
    //     .and(intakingStateMachine.stateTriggers.get(IntakingState.STOWED))
    //     .whileTrue(intake.rollers.runLinearVelocity(IntakeConstants.Rollers.AGITATING_VELOCITY));

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
        .onTrue(forceState(IntakingState.INTAKING)); // Move to appropriate state

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.STOWED)
        .whileFalse( // Run the intake based on drivetrain speed
            intake.runRollers(drive::getRotation, drive::getChassisSpeeds));
  }

  private void configureGameStateTriggers() {
    new Trigger(() -> DriverStation.isTeleopEnabled()).onTrue(forceState(ShootingState.IDLE));
    // .onTrue(forceState(IntakingState.DEPLOYING));
  }

  private void configureShooter() {
    turret.setDefaultCommand(
        turret.lockOntoTarget( // Have the turret track the target
            () -> ShootingUtil.calculateTurretRelativeAngle(drive.getPose(), shootingTarget),
            () ->
                ShootingUtil.getAngularVelocityCompensation(
                    drive.getPose(), hubPoseSupplier.get(), drive.getChassisSpeeds())));

    hood.setDefaultCommand(
        hood.trackTarget(() -> ShootingUtil.calculateHoodAngle(distanceFromTargetMeters)));

    passingModeTrigger
        .whileTrue(
            turret.lockOntoTarget(
                () ->
                    DriveConstants.getZeroOrientation()
                        .plus(Rotation2d.k180deg)
                        .minus(drive.getRotation())
                        .getMeasure()))
        .whileTrue(hood.runTargetAngle(() -> Degrees.of(26.5)));

    manualOverrideTrigger
        .and(operatorController.povUp().negate())
        .whileTrue(turret.manualControl(() -> -operatorController.getRightX()))
        .whileTrue(hood.manualControl(() -> -operatorController.getLeftY()));

    manualOverrideTrigger
        .and(operatorController.povUp())
        // Helper button for zeroing before restarting bot
        .whileTrue(turret.runTargetAngle(() -> Rotations.zero()));
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

  private Command restartSerializerAntiStalled() {
    return new SequentialCommandGroup(
        new PrintCommand("Anti-stalling Activated"),
        serializer.setAngularVelocity(RotationsPerSecond.of(-0.5)),
        new WaitUntilCommand(() -> !serializer.isStalling()).raceWith(new WaitCommand(1.0)),
        serializer
            .startSerializer()
            .onlyIf(() -> shootingStateMachine.isInState(ShootingState.SHOOTING)),
        new PrintCommand("Anti-stalling Stopped"));
  }

  public SequentialCommandGroup intakePitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(),
        intake.currentSensedRezero(),
        forceState(IntakingState.DEPLOYING),
        new WaitCommand(3.0),
        forceState(IntakingState.STOWING));
  }

  public SequentialCommandGroup hoodPitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(),
        hood.currentSensedRezero(),
        hood.setTargetAngle(Degrees.of(0)),
        new WaitCommand(1.0),
        hood.setTargetAngle(Degrees.of(15)),
        new WaitCommand(1.0),
        hood.setTargetAngle(Degrees.of(26.5)),
        new WaitCommand(1.0),
        hood.setTargetAngle(Degrees.of(5)),
        new WaitCommand(1.0),
        hood.setTargetAngle(Degrees.of(0)),
        new WaitCommand(4.0));
  }

  public SequentialCommandGroup balltunnelerPitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(),
        ballTunneler.startTunneler(),
        flywheel.setVelocity(RadiansPerSecond.of(150)),
        new WaitCommand(6.0),
        ballTunneler.stop(),
        flywheel.stop());
  }

  public SequentialCommandGroup serializerPitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(), serializer.runSerializer(), new WaitCommand(6.0), serializer.stop());
  }

  public SequentialCommandGroup flywheelPitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(),
        flywheel.setVelocity(RadiansPerSecond.of(560)),
        new WaitCommand(6.0),
        flywheel.stop());
  }

  public SequentialCommandGroup shootingPitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(),
        flywheel.setVelocity(RadiansPerSecond.of(150)),
        serializer.startSerializer(),
        ballTunneler.startTunneler(),
        new WaitCommand(6.0),
        flywheel.stop(),
        ballTunneler.stop(),
        serializer.stop());
  }

  public SequentialCommandGroup turretPitCheck() {
    return new SequentialCommandGroup(
        startManualOverride(),
        turret.setTargetAngle(Degrees.of(90)),
        new WaitCommand(1.0),
        turret.setTargetAngle(Degrees.of(180)),
        new WaitCommand(1.0),
        turret.setTargetAngle(Degrees.of(0)),
        new WaitCommand(1.0),
        turret.setTargetAngle(Degrees.of(-90)),
        new WaitCommand(1.0),
        turret.setTargetAngle(Degrees.of(-180)),
        new WaitCommand(1.0),
        turret.setTargetAngle(Degrees.of(0)),
        new WaitCommand(4.0));
  }

  public Command toggleBumpMode() {
    return runOnce(
        () -> {
          if (drive.getMaxLinearSpeed().equals(DriveConstants.shootingModeMaxSpeed)) {
            if (shootingStateMachine.isInState(ShootingState.IDLE)) {
              drive.setMaxLinearSpeed(TunerConstants.kSpeedAt12Volts);
            } else {
              drive.setMaxLinearSpeed(DriveConstants.shootingModeMaxSpeed);
            }
          } else {
            drive.setMaxLinearSpeed(DriveConstants.shootingModeMaxSpeed);
          }
        });
  }

  public Command toggleIntakeExtension() {
    return Commands.runOnce(
        () -> {
          if (intakingStateMachine.isInState(IntakingState.STOWING)
              || intakingStateMachine.isInState(IntakingState.STOWED)) {
            intakingStateMachine.requestState(IntakingState.DEPLOYING);
          } else {
            intakingStateMachine.requestState(IntakingState.STOWING);
          }
        });
  }

  public Command toggleManualOverride() {
    return Commands.runOnce(() -> isManualOverride = !isManualOverride);
  }

  public Command startManualOverride() {
    return Commands.runOnce(() -> isManualOverride = true);
  }

  public Command stopManualOverride() {
    return Commands.runOnce(() -> isManualOverride = false);
  }

  private Trigger shouldStopSerializer() {
    return shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(new Trigger(serializer::isStalling));
  }

  @Override
  public void periodic() {
    Pose2d drivePose = drive.getPose();

    shootingTarget = // hubPoseSupplier.get();
        ShootingUtil.correctTargetPoseWhileMoving(
            drivePose, hubPoseSupplier.get(), drive.getFieldOrientedSpeeds());

    distanceFromTargetMeters =
        shootingTarget.getTranslation().getDistance(drivePose.getTranslation());

    isPassing = FieldConstants.shouldBePassing(drivePose);

    Logger.recordOutput("Superstructure/ShootingState", shootingStateMachine.getState().toString());
    Logger.recordOutput("Superstructure/IntakingState", intakingStateMachine.getState().toString());

    // Render a Pose showing where the turret (thinks it) is pointing
    Logger.recordOutput(
        "RobotRendering/TurretHeading",
        drive
            .getPose()
            .plus(new Transform2d(0, 0, new Rotation2d(turret.getOrientation())))
            .plus(new Transform2d(distanceFromTargetMeters, 0, Rotation2d.kZero)));
  }
}
