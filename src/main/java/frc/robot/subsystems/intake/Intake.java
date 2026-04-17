package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public final Rollers rollers;
  private final LoggedTunableNumber rollersKv =
      new LoggedTunableNumber("Intake/kV", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber rollersKa =
      new LoggedTunableNumber("Intake/kA", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber rollersKs =
      new LoggedTunableNumber("Intake/kS", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber rollersKp =
      new LoggedTunableNumber("Intake/kP", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber rollersKd =
      new LoggedTunableNumber("Intake/kD", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kD);

  public final Extension extension;
  private final LoggedTunableNumber extensionKs =
      new LoggedTunableNumber("IntakePivot/kS", IntakeConstants.Extension.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber extensionKv =
      new LoggedTunableNumber("IntakePivot/kV", IntakeConstants.Extension.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber extensionKa =
      new LoggedTunableNumber("IntakePivot/kA", IntakeConstants.Extension.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber extensionKp =
      new LoggedTunableNumber("IntakePivot/kP", IntakeConstants.Extension.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber extensionKd =
      new LoggedTunableNumber("IntakePivot/kD", IntakeConstants.Extension.SYSTEM_CONSTANTS.kD);
  private final LoggedTunableNumber extensionMaxVelocity =
      new LoggedTunableNumber(
          "IntakePivot/maxVelocity", IntakeConstants.Extension.SYSTEM_CONSTANTS.maxVelocity.get());
  private final LoggedTunableNumber extensionMaxAcceleration =
      new LoggedTunableNumber(
          "IntakePivot/maxAcceleration",
          IntakeConstants.Extension.SYSTEM_CONSTANTS.maxAcceleration.get());

  private final Debouncer currentSenseDebouncer = new Debouncer(0.04);
  private final Debouncer intakeDebouncer = new Debouncer(0.1, DebounceType.kBoth);

  public Intake(RollersIO rollersIO, ExtensionIO extensionIO) {
    this.rollers = new Rollers("Intake/Rollers", rollersIO);
    this.extension = new Extension("Intake/Extension", extensionIO);

    this.extension.setSetpointTolerance(IntakeConstants.Extension.SETPOINT_TOLERANCE);
    // extensionIO.setExtraGain(0.1, IntakeConstants.Extension.SETPOINT_TOLERANCE);

    rollersIO.setControlConstants(
        rollersKs.get(), rollersKv.get(), rollersKa.get(), rollersKp.get(), rollersKd.get());
    extensionIO.setControlConstants(
        extensionKs.get(),
        extensionKv.get(),
        extensionKa.get(),
        extensionKp.get(),
        extensionKd.get());
    extensionIO.setMotionProfile(extensionMaxVelocity.get(), extensionMaxAcceleration.get());
  }

  public Command stow() {
    return extension.setPosition(IntakeConstants.Extension.STOWED_POSITION);
  }

  public Command deploy() {
    return extension.setPosition(IntakeConstants.Extension.DEPLOYED_POSITION);
  }

  public Command halfStow() {
    return extension.setPosition(IntakeConstants.Extension.PARTIAL_STOWED_POSITION);
  }

  @AutoLogOutput(key = "Intake/IsIntakingInAuto")
  public boolean isIntakingInAuto() {
    return intakeDebouncer.calculate(rollers.inputs.supplyCurrentAmps > 20);
  }

  public Command currentSensedRezero(double timeoutSeconds) {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new SequentialCommandGroup(
              this.runOnce(
                  () -> {
                    Logger.recordOutput("Intake/Rezeroing", true);
                    extension.shouldAutoStopAtSetpoint = false;
                    extension.io.setVolts(-3);
                  }),
              this.runOnce(() -> rollers.io.stop()),
              new WaitUntilCommand(
                      () ->
                          currentSenseDebouncer.calculate(
                              Math.abs(extension.inputs.torqueCurrentAmps) > 70))
                  .raceWith(new WaitCommand(timeoutSeconds)),
              runOnce(() -> extension.io.seedPosition(Inches.of(-0.3))),
              this.runOnce(() -> extension.io.stop()))
          .finallyDo(
              () -> {
                extension.shouldAutoStopAtSetpoint = true;
                Logger.recordOutput("Intake/Rezeroing", false);
              });
    } else {
      return this.runOnce(() -> rollers.io.stop()).andThen(new WaitCommand(0.5));
    }
  }

  public Command runRollers(
      Supplier<Rotation2d> driveHeadingSupplier, Supplier<ChassisSpeeds> drivetrainSpeedsSupplier) {
    return rollers.runLinearVelocity(
        () -> {
          ChassisSpeeds robotSpeeds = drivetrainSpeedsSupplier.get();

          // Just use minimum speed when not moving
          if (robotSpeeds.vxMetersPerSecond == 0 && robotSpeeds.vyMetersPerSecond == 0)
            return IntakeConstants.Rollers.MINIMUM_INTAKE_SPEED;

          Rotation2d driverHeading = driveHeadingSupplier.get();
          ChassisSpeeds fieldOrientedSpeeds =
              ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, driverHeading);
          Translation2d dtSpeedsVector =
              new Translation2d(
                  fieldOrientedSpeeds.vxMetersPerSecond, fieldOrientedSpeeds.vyMetersPerSecond);
          LinearVelocity totalDTSpeeds = MetersPerSecond.of(dtSpeedsVector.getNorm());

          LinearVelocity alignedDTSpeeds =
              totalDTSpeeds.times(
                  2 * Math.max(dtSpeedsVector.getAngle().minus(driverHeading).getCos(), 0));

          LinearVelocity totalSpeeds =
              alignedDTSpeeds.plus(IntakeConstants.Rollers.MINIMUM_INTAKE_SPEED);

          return totalSpeeds;
        });
  }

  @Override
  public void periodic() {
    int id = hashCode();

    // LoggedTunableNumber.ifChanged(
    //     id,
    //     c -> rollers.io.setControlConstants(c[0], c[1], c[2], c[3], c[4]),
    //     rollersKs,
    //     rollersKv,
    //     rollersKa,
    //     rollersKp,
    //     rollersKd);
    LoggedTunableNumber.ifChanged(
        id,
        c -> extension.io.setControlConstants(c[0], c[1], c[2], c[3], c[4]),
        extensionKs,
        extensionKv,
        extensionKa,
        extensionKp,
        extensionKd);
    LoggedTunableNumber.ifChanged(
        id,
        c -> extension.io.setMotionProfile(c[0], c[1]),
        extensionMaxVelocity,
        extensionMaxAcceleration);
  }
}
