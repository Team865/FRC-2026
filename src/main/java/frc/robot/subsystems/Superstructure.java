package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PitCheck;
import frc.robot.util.Shooting.ShootingLogger;
import frc.robot.util.ShootingUtilLegacy;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
    /** Requestable: The intake is partially stowed to prevent fuel from leaking */
    PARTIAL_STOW,
    /** Intermediate: The intake is deployed and running */
    DEPLOYED
  }

  public static enum PassingSide {
    LEFT,
    RIGHT
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

  private Angle turretTargetAngle = Rotations.zero();
  private Angle hoodTargetAngle = Rotations.zero();
  private AngularVelocity flywheelTargetVelocity = RotationsPerSecond.zero();

  @AutoLogOutput(key = "Superstructure/DistanceFromShootingGoalMeters")
  private double distanceFromTargetMeters = 0.0;

  private boolean isPassing = false;
  private boolean isSlowMode = false;
  private boolean pitCheckMode = false;

  @AutoLogOutput(key = "Superstructure/IsOutaking")
  private boolean isOutaking = false;

  private int numStallsDetected = 0;

  private final CommandXboxController operatorController;

  @AutoLogOutput(key = "Superstructure/ManualOverride")
  private final Trigger manualOverrideTrigger = new Trigger(() -> isManualOverride);

  @AutoLogOutput(key = "Superstructure/PitCheck")
  private final Trigger pitCheckTrigger = new Trigger(() -> pitCheckMode);

  @AutoLogOutput(key = "Superstructure/PassingMode")
  private final Trigger passingModeTrigger =
      new Trigger(() -> isPassing && DriverStation.isTeleopEnabled() && !isManualOverride);

  @AutoLogOutput(key = "Superstructure/PassingSide")
  private PassingSide passingSide;

  private static final LoggedTunableNumber flywheelTestVelocityRadsPerSec =
      new LoggedTunableNumber("Testing/FlywheelTestVelocityRadsPerSec", 350.0);

  private static final LoggedTunableNumber hoodTestAngleDeg =
      new LoggedTunableNumber("Testing/HoodTestAngleDeg", 0.0);

  private static final ShootingLogger SHOOTING_LOGGER = new ShootingLogger();

  public Superstructure(
      Drive drive,
      Intake intake,
      Serializer serializer,
      BallTunneler ballTunneler,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      LEDs leds,
      CommandXboxController driverController,
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

    PitCheck.registerSuperstructure(this);

    configureStateRequirements();
    configureStateBehaviours();
    configureGameStateTriggers();
    configureShooter();

    this.passingSide =
        FieldConstants.isOnRightSide(drive.getPose()) ? PassingSide.RIGHT : PassingSide.LEFT;

    driverController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () ->
                    SHOOTING_LOGGER.addMeasurement(
                        distanceFromTargetMeters,
                        hoodTestAngleDeg.get(),
                        flywheelTestVelocityRadsPerSec.get())));
    driverController.back().onTrue(Commands.runOnce(() -> SHOOTING_LOGGER.save()));
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
        IntakingState.DEPLOYED, StateMachine.STATE_ALWAYS_AVAILABLE);
    intakingStateMachine.stateRequirements.put(
        IntakingState.PARTIAL_STOW, () -> intakingStateMachine.isInState(IntakingState.DEPLOYED));
  }

  /** Configure what the behaviours of each state are */
  private void configureStateBehaviours() {
    // Stop the flywheels when in Idle shooting state
    // shootingStateMachine.stateTriggers.get(ShootingState.IDLE).onTrue(leds.idleWaveCommand());
    // shootingStateMachine
    //     .stateTriggers
    //     .get(ShootingState.SHOOTING)
    //

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(passingModeTrigger)
        .whileTrue(
            flywheel.runVelocity(
                () ->
                    ShootingUtilLegacy.getPassingFlywheelVelocity(
                        Math.abs(
                            drive.getPose().getX() - FieldConstants.Passing.getBumpLineXPos()))));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(new Trigger(() -> DriverStation.isTeleopEnabled()))
        .onFalse(
            drive
                .setMaxLinearSpeedCmd(TunerConstants.kSpeedAt12Volts)
                .onlyIf(() -> !isSlowMode)
                .alongWith(leds.shootingIdleWaveCommand()))
        .onTrue(
            drive
                .setMaxLinearSpeedCmd(DriveConstants.shootingModeMaxSpeed)
                .alongWith(leds.shootingActiveWaveCommand()));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(new Trigger(() -> DriverStation.isAutonomous()))
        .onTrue(leds.autoShootingActiveWaveCommand())
        .onFalse(leds.idleWaveCommand());

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(manualOverrideTrigger.or(turret.canShoot()))
        .whileTrue(ballTunneler.runTunneler())
        .onTrue(
            new WaitCommand(0.25)
                .andThen(
                    serializer
                        .startSerializer()
                        .onlyIf(() -> shootingStateMachine.isInState(ShootingState.SHOOTING))))
        .onFalse(serializer.stop());

    // Intaking state
    intakingStateMachine
        .stateTriggers
        .get(IntakingState.STOWING)
        .onTrue(intake.stow()) // Stow the intake
        .and(intake.extensionAtSetpoint()) // If the intake is stowed,
        .onTrue(forceState(IntakingState.STOWED)); // move to appropriate state

    intakingStateMachine.stateTriggers.get(IntakingState.DEPLOYING).onTrue(intake.deploy());
    intakingStateMachine.stateTriggers.get(IntakingState.PARTIAL_STOW).onTrue(intake.halfStow());

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.DEPLOYING) // Deploy the intake
        .and(intake.extensionAtSetpoint()) // If the intake arm is deployed,
        .onTrue(forceState(IntakingState.DEPLOYED)); // Move to appropriate state

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.STOWED)
        .whileTrue( // Run the intake based on drivetrain speed
            intake.rollers.runVolts(12.0));

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.DEPLOYED)
        .whileTrue( // Run the intake based on drivetrain speed
            intake.rollers.runVolts(12.0));
    intakingStateMachine
        .stateTriggers
        .get(IntakingState.DEPLOYING)
        .whileTrue( // Run the intake based on drivetrain speed
            intake.rollers.runVolts(12.0));
    intakingStateMachine
        .stateTriggers
        .get(IntakingState.STOWING)
        .whileTrue( // Run the intake based on drivetrain speed
            intake.rollers.runVolts(12.0));
    intakingStateMachine
        .stateTriggers
        .get(IntakingState.PARTIAL_STOW)
        .whileTrue( // Run the intake based on drivetrain speed
            intake.rollers.runVolts(12.0));
  }

  private void configureGameStateTriggers() {
    new Trigger(DriverStation::isTeleopEnabled).onTrue(forceState(ShootingState.IDLE));

    new Trigger(DriverStation::isAutonomousEnabled)
        .negate()
        .onTrue(Commands.runOnce(() -> isManualOverride = false).ignoringDisable(true));
  }

  private void configureShooter() {
    Trigger shouldAutoAim = manualOverrideTrigger.or(pitCheckTrigger).negate();

    turret.setDefaultCommand(
        turret
            .lockOntoTarget( // Have the turret track the target
                () -> turretTargetAngle,
                () ->
                    ShootingUtilLegacy.getAngularVelocityCompensation(
                        drive.getPose(), hubPoseSupplier.get(), drive.getChassisSpeeds()))
            .onlyIf(shouldAutoAim)
            .onlyWhile(shouldAutoAim));

    hood.setDefaultCommand(
        hood.trackTarget(() -> hoodTargetAngle).onlyIf(shouldAutoAim).onlyWhile(shouldAutoAim));

    flywheel.setDefaultCommand(
        flywheel
            .runVelocity(() -> flywheelTargetVelocity)
            .onlyIf(pitCheckTrigger.negate())
            .onlyWhile(pitCheckTrigger.negate()));

    passingModeTrigger.whileTrue(hood.runTargetAngle(() -> Degrees.of(26.5)));

    manualOverrideTrigger
        .and(operatorController.povUp().negate())
        .and(DriverStation::isTeleopEnabled)
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

  public Command setPassingSide(PassingSide side) {
    return Commands.runOnce(() -> this.passingSide = side);
  }

  private Command restartSerializerAntiStalled() {
    return new SequentialCommandGroup(
        Commands.runOnce(
            () -> {
              numStallsDetected++;
              Logger.recordOutput("Serializer/StallsDetected", numStallsDetected);
            }),
        new PrintCommand("Anti-stalling Activated"),
        // serializer.setVolts(-2.0),
        new WaitUntilCommand(() -> !serializer.isStalling()).raceWith(new WaitCommand(0.05)),
        // serializer
        //     .startSerializer()
        //     .onlyIf(() -> shootingStateMachine.isInState(ShootingState.SHOOTING)),
        new PrintCommand("Anti-stalling Stopped"));
  }

  public Command intakePitCheck() {
    Distance[] extensionSetpoints = {
      IntakeConstants.Extension.STOWED_POSITION,
      IntakeConstants.Extension.PARTIAL_STOWED_POSITION,
      IntakeConstants.Extension.DEPLOYED_POSITION,
      IntakeConstants.Extension.STOWED_POSITION
    };

    LinearVelocity[] rollersSetpoints = {
      MetersPerSecond.of(2.0),
      MetersPerSecond.of(4.0),
      MetersPerSecond.of(6.0),
      MetersPerSecond.of(7.0)
    };

    return new SequentialCommandGroup(
            startManualOverride(),
            intake.currentSensedRezero(),
            PitCheck.createCommand(
                "Intake Extension Pit Check",
                intake.extension.io::setPosition,
                intake.extension::isAtSetpoint,
                1.0,
                5.0,
                extensionSetpoints),
            PitCheck.createCommand(
                "Intake Rollers Pit Check",
                intake.rollers.io::setLinearVelocity,
                intake.rollers::isAtSetpoint,
                1.0,
                5.0,
                rollersSetpoints))
        .finallyDo(
            () -> {
              intake.extension.stop();
              intake.rollers.stop();
            });
  }

  public Command hoodPitCheck() {
    Angle[] setpoints = {
      Degrees.of(0),
      Degrees.of(5),
      Degrees.of(10),
      Degrees.of(15),
      Degrees.of(20),
      Degrees.of(26.5),
      Degrees.of(0.0)
    };

    return startManualOverride()
        .andThen(
            hood.currentSensedRezero()
                .andThen(
                    PitCheck.createCommand(
                        "Hood Pit Checks",
                        hood.io::setPosition,
                        hood::isAtSetpoint,
                        1,
                        5,
                        setpoints,
                        hood)));
  }

  public Command balltunnelerPitCheck() {
    AngularVelocity[] setpoints = {
      RadiansPerSecond.of(100),
      RadiansPerSecond.of(150),
      RadiansPerSecond.of(200),
      RadiansPerSecond.of(250),
      RadiansPerSecond.of(300),
      RadiansPerSecond.zero()
    };

    return new SequentialCommandGroup(
            flywheel.setVelocity(RadiansPerSecond.of(150)),
            PitCheck.createCommand(
                "Ball Tunneler Pit Check",
                ballTunneler.io::setAngularVelocity,
                ballTunneler::isAtSetpoint,
                1.0,
                5.0,
                setpoints,
                ballTunneler))
        .finallyDo(flywheel.io::stop);
  }

  public Command serializerPitCheck() {
    AngularVelocity[] setpoints = {
      RotationsPerSecond.of(0.5), RotationsPerSecond.of(1.0), RotationsPerSecond.of(1.5)
    };

    return PitCheck.createCommand(
        "Serializer Pit Check",
        serializer.io::setAngularVelocity,
        serializer::isAtSetpoint,
        1.0,
        5.0,
        setpoints,
        serializer);
  }

  public Command turretPitCheck() {
    Angle[] setpoints = {
      Degrees.of(0),
      Degrees.of(90),
      Degrees.of(0),
      Degrees.of(180),
      Degrees.of(0),
      Degrees.of(-90),
      Degrees.of(0),
      Degrees.of(-180),
      Degrees.of(0)
    };

    return PitCheck.createCommand(
        "Turret Pit Checks",
        turret.io::setPosition,
        turret::isAtSetpoint,
        0.5,
        5.0,
        setpoints,
        turret);
  }

  public Command tunnelerShootingPitCheck() {
    return new SequentialCommandGroup(
            startManualOverride(),
            flywheel.setVelocity(RadiansPerSecond.of(150)),
            ballTunneler.startTunneler(),
            new WaitCommand(15.0))
        .finallyDo(
            () -> {
              System.out.println(ballTunneler.getAngularVelocity().toString());
              System.out.println(flywheel.getAngularVelocity().toString());
              flywheel.io.stop();
              ballTunneler.io.stop();
              isManualOverride = false;
            });
  }

  public Command fullShootingPitCheck() {
    return new SequentialCommandGroup(
            startManualOverride(),
            flywheel.setVelocity(RadiansPerSecond.of(180.0)),
            ballTunneler.startTunneler(),
            new WaitCommand(0.5),
            serializer.startSerializer(),
            new WaitCommand(15.0))
        .finallyDo(
            () -> {
              System.out.println(serializer.getAngularVelocity().toString());
              System.out.println(ballTunneler.getAngularVelocity().toString());
              System.out.println(flywheel.getAngularVelocity().toString());
              flywheel.io.stop();
              ballTunneler.io.stop();
              serializer.io.stop();
              isManualOverride = false;
            });
  }

  public Command toggleSlowMode() {
    return runOnce(
            () -> {
              isSlowMode = !isSlowMode;
              drive.setMaxLinearSpeed(
                  isSlowMode
                      ? DriveConstants.shootingModeMaxSpeed
                      : TunerConstants.kSpeedAt12Volts);
            })
        .ignoringDisable(true);
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

  public Command runOutake() {
    return Commands.runEnd(() -> isOutaking = true, () -> isOutaking = false).ignoringDisable(true);
  }

  private Trigger shouldStopSerializer() {
    return shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .and(serializer::isStalling);
  }

  @Override
  public void periodic() {
    Pose2d drivePose = drive.getPose();

    isPassing = FieldConstants.Passing.shouldBePassing(drivePose);
    passingSide = FieldConstants.isOnRightSide(drivePose) ? PassingSide.RIGHT : PassingSide.LEFT;

    Pose2d originalGoal =
        (isPassing && DriverStation.isTeleopEnabled())
            ? ((FieldConstants.isOnRightSide(drivePose)
                ? FieldConstants.Passing.getRightCorner()
                : FieldConstants.Passing.getLeftCorner()))
            : hubPoseSupplier.get();

    // if (false) {
    //   ShootingCalculation shotCalculation =
    //       ShotCalculator.calculate(drivePose, originalGoal, drive.getFieldOrientedSpeeds());

    //   turretTargetAngle = shotCalculation.yaw();
    //   hoodTargetAngle = shotCalculation.pitch();
    //   flywheelTargetVelocity = shotCalculation.flywheelVelocity();
    //   Logger.recordOutput("Superstructure/ShooterTarget", shotCalculation.virtualGoal());
    // }

    // Legacy Shot Calc
    hoodTargetAngle = Degrees.of(hoodTestAngleDeg.get());
    flywheelTargetVelocity = RadiansPerSecond.of(flywheelTestVelocityRadsPerSec.get());

    if (!isPassing) {
      turretTargetAngle = ShootingUtilLegacy.calculateTurretRelativeAngle(drivePose, originalGoal);

      // Pose2d virtualGoal =
      //     ShootingUtilLegacy.correctTargetPoseWhileMoving(
      //         drivePose, originalGoal, drive.getChassisSpeeds());
      // distanceFromTargetMeters =
      //     virtualGoal.getTranslation().getDistance(drivePose.getTranslation());

      // turretTargetAngle = ShootingUtilLegacy.calculateTurretRelativeAngle(drivePose,
      // virtualGoal);
      // hoodTargetAngle = ShootingUtilLegacy.calculateHoodAngle(distanceFromTargetMeters);
      // flywheelTargetVelocity =
      //     ShootingUtilLegacy.getScoringFlywheelVelocity(distanceFromTargetMeters);
    } else {
      turretTargetAngle =
          ((AllianceFlipUtil.shouldFlip() ? Rotation2d.kZero : Rotation2d.k180deg)
                  .minus(drivePose.getRotation()))
              .getMeasure();
    }

    Logger.recordOutput(
        "Superstructure/DistanceFromBumpMeters",
        Math.abs(drivePose.getX() - FieldConstants.Passing.getBumpLineXPos()));
    Logger.recordOutput("Superstructure/ShootingState", shootingStateMachine.getState().toString());
    Logger.recordOutput("Superstructure/IntakingState", intakingStateMachine.getState().toString());
    Logger.recordOutput("Superstructure/SlowMode", isSlowMode);

    // // Render a Pose showing where the turret (thinks it) is pointing
    // Logger.recordOutput(
    //     "RobotRendering/TurretHeading",
    //     drive
    //         .getPose()
    //         .plus(new Transform2d(0, 0, new Rotation2d(turret.getOrientation())))
    //         .plus(new Transform2d(distanceFromTargetMeters, 0, Rotation2d.kZero)));
  }
}
