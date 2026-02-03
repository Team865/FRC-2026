// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.pivot.PivotIOTalonFX.PivotTalonFXConstants;
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
  private final Intake intake;
  private final Vision vision;

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
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
        intake =
            new Intake(
                new RollersIOTalonFX(
                    IntakeConstants.ROLLER_CAN_ID, IntakeConstants.CANBUS, IntakeConstants.ROLLERS),
                new PivotIOTalonFX(
                    IntakeConstants.PIVOT_CAN_ID,
                    IntakeConstants.CANBUS,
                    new PivotTalonFXConstants(IntakeConstants.Pivot.getGains(), false, 1)));

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
                new PivotIOTalonFX(
                    ShooterConstants.Turret.CAN_ID,
                    ShooterConstants.Turret.CANBUS,
                    new PivotTalonFXConstants(
                        ShooterConstants.Turret.getConstants(),
                        false,
                        ShooterConstants.Turret.GEAR_RATIO)));
        hood =
            new Hood(
                new PivotIOTalonFX(
                    ShooterConstants.Hood.CAN_ID,
                    ShooterConstants.Hood.CANBUS,
                    new PivotTalonFXConstants(
                        ShooterConstants.Hood.getConstants(),
                        false,
                        ShooterConstants.Hood.GEAR_RATIO)));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
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

        intake =
            new Intake(
                new RollersIOSim(DCMotor.getKrakenX60(1), 1, IntakeConstants.ROLLERS),
                new PivotIOSim(DCMotor.getKrakenX60(1), IntakeConstants.Pivot.getGains()));

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
                new PivotIOSim(DCMotor.getKrakenX60(1), ShooterConstants.Turret.getConstants()));
        hood =
            new Hood(new PivotIOSim(DCMotor.getKrakenX44(1), ShooterConstants.Hood.getConstants()));
        flywheel = new Flywheel(new FlywheelIOSim());
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

        intake = new Intake(new RollersIO() {}, new PivotIO() {});
        vision =
            Vision.createPerCameraVision(
                drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});

        turret = new Turret(new PivotIO() {});
        hood = new Hood(new PivotIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});

        break;
    }

    this.superstructure = new Superstructure(drive, turret, () -> getAllianceHubPose());
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Test intake
    controller.y().whileTrue(intake.runVelocity(5));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // 0 degrees
    controller.x().onTrue(intake.pivot.setTargetAngle(Degrees.of(0)));

    // 90 degrees
    controller.b().onTrue(intake.pivot.setTargetAngle(Degrees.of(90)));

    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () ->
                    getAllianceHubPose()
                        .getTranslation()
                        .minus(drive.getPose().getTranslation())
                        .getAngle()));

    // Reset gyro to 0° when B button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .leftTrigger()
        .whileTrue(
            turret.lockOntoTarget(
                () -> {
                  Pose2d drivePose = this.drive.getPose();
                  Rotation2d driveHeading = drivePose.getRotation();
                  Translation2d driveToHubVector =
                      getAllianceHubPose().getTranslation().minus(drivePose.getTranslation());
                  Rotation2d pointToHubRotation =
                      new Rotation2d(driveToHubVector.getX(), driveToHubVector.getY());

                  return pointToHubRotation.minus(driveHeading).getMeasure();
                },
                () -> drive.getAngularVelocityRadPerSec()))
        .whileTrue(
            hood.trackTarget(
                () ->
                    Radians.of(
                        getAllianceHubPose()
                            .getTranslation()
                            .getDistance(drive.getPose().getTranslation()))))
        .whileTrue(flywheel.runVelocityRadPerSec(5.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
