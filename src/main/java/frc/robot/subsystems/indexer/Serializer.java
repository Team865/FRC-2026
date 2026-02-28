package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Serializer extends Rollers {
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Serializer/kV", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Serializer/kA", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Serializer/kS", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Serializer/kP", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Serializer/kD", IndexerConstants.Serializer.SYSTEM_CONSTANTS.kD);

  //   private final SysIdRoutine sysIdRoutine;

  public Serializer(RollersIO io) {
    super("Serializer", io);

    // sysIdRoutine =
    //     new SysIdBuilder(this, io::setVolts)
    //         .withDynamicStepVoltage(3.0)
    //         .withQuasistaticRampRate(0.3)
    //         .build();

    io.setControlConstants(kV.get(), kA.get(), kS.get(), kP.get(), kD.get());
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

    // LoggedTunableNumber.ifChanged(
    //     id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3], c[4]), kV, kA, kS, kP, kD);

    super.periodic();
  }
}
