// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.IntakingState;
import frc.robot.subsystems.Superstructure.ShootingState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.pivot.PivotIOTalonFXWithCANcoder;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOTalonFX;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ComponentPoseUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Climber climber;
  private final Intake intake;
  private final Serializer serializer;
  private final BallTunneler ballTunneler;
  private final Vision vision;
  private final LEDs leds;

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Log field element positions
  @AutoLogOutput(key = "currentAllianceHubPose")
  public Pose2d getAllianceHubPose() {
    return AllianceFlipUtil.apply(FieldConstants.allianceHubPose);
  }

  @AutoLogOutput(key = "currentAllianceRightClimbPose")
  public Pose2d getAllianceRightClimbPose() {
    return AllianceFlipUtil.apply(FieldConstants.allianceRightClimbPose);
  }

  @AutoLogOutput(key = "currentAllianceLeftClimbPose")
  public Pose2d getAllianceLeftClimbPose() {
    return AllianceFlipUtil.apply(FieldConstants.allianceLeftClimbPose);
  }

  @AutoLogOutput(key = "TurretTx")
  public double getTurretTx() {
    return vision != null ? vision.getTurretTxDegrees() : 0.0;
  }

  @AutoLogOutput(key = "TurretCurrentTagID")
  public int getTurretSeesHubTag() {
    return vision.getTurretSeenTagId();
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // canbus

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        climber = new Climber(new ClimberIO() {});
        intake =
            new Intake(
                new RollersIOTalonFX(
                    IntakeConstants.Rollers.CAN_ID,
                    IntakeConstants.CANBUS,
                    IntakeConstants.Rollers.SPECS),
                new PivotIOTalonFXWithCANcoder(
                    IntakeConstants.Pivot.MOTOR_CAN_ID,
                    IntakeConstants.Pivot.CANCODER_ID,
                    IntakeConstants.CANBUS,
                    IntakeConstants.Pivot.PIVOT_SPECS,
                    IntakeConstants.Pivot.CANCODER_SPECS));

        serializer =
            new Serializer(
                new RollersIOTalonFX(
                    IndexerConstants.Serializer.CAN_ID,
                    IndexerConstants.CANBUS,
                    IndexerConstants.Serializer.ROLLERS_SPECS));
        ballTunneler =
            new BallTunneler(
                new RollersIOTalonFX(
                    IndexerConstants.BallTunneler.CAN_ID,
                    IndexerConstants.CANBUS,
                    IndexerConstants.BallTunneler.ROLLERS_SPECS));
        vision =
            Vision.createPerCameraVision(
                drive,
                new VisionIOLimelight(
                    VisionConstants.camera0Name, () -> drive.getPose().getRotation(), true),
                new VisionIOLimelight(
                    VisionConstants.camera1Name, () -> drive.getPose().getRotation(), true),
                new VisionIOLimelight(
                    VisionConstants.camera2Name, () -> drive.getPose().getRotation(), false));

        turret =
            new Turret(
                new PivotIOTalonFXWithCANcoder(
                    ShooterConstants.Turret.MOTOR_ID,
                    ShooterConstants.Turret.CANCODER_ID,
                    ShooterConstants.CANBUS,
                    ShooterConstants.Turret.PIVOT_SPECS,
                    ShooterConstants.Turret.CANCODER_SPECS));
        hood =
            new Hood(
                new PivotIOTalonFX(
                    ShooterConstants.Hood.CAN_ID,
                    ShooterConstants.CANBUS,
                    ShooterConstants.Hood.SPECS));
        flywheel = new Flywheel(new FlywheelIOTalonFX());

        leds = new LEDs();
        break;

      case SIM:

        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        climber = new Climber(new ClimberIOSim());
        intake =
            new Intake(
                new RollersIOSim(
                    DCMotor.getKrakenX60(1),
                    IntakeConstants.Rollers.SYSTEM_CONSTANTS,
                    IntakeConstants.Rollers.SPECS),
                new PivotIOSim(DCMotor.getKrakenX60(1), IntakeConstants.Pivot.SYSTEM_CONSTANTS));

        serializer =
            new Serializer(
                new RollersIOSim(
                    DCMotor.getKrakenX60(1),
                    IndexerConstants.Serializer.SYSTEM_CONSTANTS,
                    IndexerConstants.Serializer.ROLLERS_SPECS));
        ballTunneler =
            new BallTunneler(
                new RollersIOSim(
                    DCMotor.getKrakenX44(1),
                    IndexerConstants.BallTunneler.SYSTEM_CONSTANTS,
                    IndexerConstants.BallTunneler.ROLLERS_SPECS));
        vision =
            Vision.createPerCameraVision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    () -> drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToCamera1,
                    () -> drive.getPose()),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name,
                    VisionConstants.robotToCamera2,
                    () -> drive.getPose()));
        turret =
            new Turret(
                new PivotIOSim(DCMotor.getKrakenX60(1), ShooterConstants.Turret.SYSTEM_CONSTANTS));
        hood =
            new Hood(
                new PivotIOSim(DCMotor.getKrakenX44(1), ShooterConstants.Hood.SYSTEM_CONSTANTS));
        flywheel = new Flywheel(new FlywheelIOSim());

        leds = new LEDs();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        climber = new Climber(new ClimberIO() {});
        intake = new Intake(new RollersIO() {}, new PivotIO() {});
        vision =
            Vision.createPerCameraVision(
                drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});

        turret = new Turret(new PivotIO() {});
        hood = new Hood(new PivotIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        serializer = new Serializer(new RollersIO() {});
        leds = new LEDs();
        ballTunneler = new BallTunneler(new RollersIO() {});

        break;
    }
    NamedCommands.registerCommand("StowIntake", intake.pivot.setTargetAngle(Degrees.of(0)));
    NamedCommands.registerCommand("DeployIntake", intake.pivot.setTargetAngle(Degrees.of(-90)));
    this.superstructure =
        new Superstructure(
            drive,
            intake,
            serializer,
            ballTunneler,
            turret,
            hood,
            flywheel,
            () -> getAllianceHubPose());
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // DRIVE CONTROLLER
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Reset gyro to 0° when B button is pressed
    driverController.start().onTrue(DriveCommands.resetGyro(drive).ignoringDisable(true));

    driverController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.k180deg));
    driverController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kCCW_90deg));
    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kCW_90deg));

    driverController
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> {
                  Translation2d lookatVector =
                      getAllianceHubPose().getTranslation().minus(drive.getPose().getTranslation());

                  return new Rotation2d(lookatVector.getX(), lookatVector.getY());
                }));

    // Shooting state reset
    driverController.back().onTrue(superstructure.forceState(ShootingState.IDLE));

    // Toggle bump mode
    driverController.rightBumper().onTrue(superstructure.toggleBumpMode());

    // Toggle shooting mode
    driverController.leftBumper().onTrue(superstructure.toggleShootingMode());

    // Start shooting and stop when let go
    driverController
        .rightTrigger()
        .whileTrue(superstructure.continuouslyRequestState(ShootingState.SHOOTING))
        .onFalse(superstructure.requestState(ShootingState.READY_TO_SHOOT));

    // Climb
    driverController.povUp().onTrue(climber.extend());
    driverController.povDown().onTrue(climber.retract());

    // OPERATOR CONTROLLER
    // State reset
    operatorController.back().onTrue(superstructure.requestState(IntakingState.STOWED));

    // Intake Slapdown
    operatorController.a().onTrue(superstructure.toggleIntakeArm());

    // Intake Rollers
    operatorController.y().onTrue(superstructure.requestState(IntakingState.INTAKING));
    operatorController.b().onTrue(superstructure.requestState(IntakingState.INTAKE_READY));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateComponentPoses() {
    ComponentPoseUtil.publishComponentPoses(serializer, turret, intake.pivot, climber);
  }
}
