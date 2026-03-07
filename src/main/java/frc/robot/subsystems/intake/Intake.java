package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.extension.ExtensionIO;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

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

  public Intake(RollersIO rollersIO, ExtensionIO extensionIO) {
    this.rollers = new Rollers("Intake/Rollers", rollersIO);
    this.extension = new Extension("Intake/Extension", extensionIO);
  }

  public Command stow() {
    return extension.setPosition(IntakeConstants.Extension.STOWED_POSITION);
  }

  public Command deploy() {
    return extension.setPosition(IntakeConstants.Extension.DEPLOYED_POSITION);
  }

  public Trigger extensionAtSetpoint() {
    return extension.atSetpoint();
  }

  public Command runRollers(Supplier<ChassisSpeeds> drivetrainSpeedsSupplier) {
    return rollers.runLinearVelocity(
        () -> {
          ChassisSpeeds speeds = drivetrainSpeedsSupplier.get();

          return MetersPerSecond.of(
              Math.max(
                  0.5,
                  2
                      * Math.sqrt(
                          speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                              + speeds.vyMetersPerSecond
                              + speeds.vyMetersPerSecond)));
        });
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id,
        c -> rollers.io.setControlConstants(c[0], c[1], c[2], c[3], c[4]),
        rollersKs,
        rollersKv,
        rollersKa,
        rollersKp,
        rollersKd);
    LoggedTunableNumber.ifChanged(
        id,
        c -> extension.io.setControlConstants(c[0], c[1], c[2], c[3], c[4]),
        extensionKs,
        extensionKv,
        extensionKa,
        extensionKp,
        extensionKd);
  }
}
