package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;

public class Serializer extends Rollers implements SysIdTestable {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Serializer/kS", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Serializer/kV", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Serializer/kP", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Serializer/kD", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kD);

  private final SysIdRoutine sysIdRoutine;

  public Serializer(RollersIO io) {
    super("Serializer", io);

    sysIdRoutine = new SysIdBuilder(this, io::setVolts).build();

    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  public Command runSerializer() {
    return runEnd(
        () ->
            io.setAngularVelocity(
                IndexerConstants.Serializer.SPINDEXING_SPEED.in(RadiansPerSecond)),
        () -> io.stop());
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);

    super.periodic();
  }

  @Override
  public SysIdRoutine getRoutine() {
    return sysIdRoutine;
  }
}
