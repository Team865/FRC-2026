package frc.robot.subsystems.serializer;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Serializer extends Rollers {

  private final RollersIO io;

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Serializer/kS", SerializerConstants.ROLLER_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Serializer/kV", SerializerConstants.ROLLER_CONSTANTS.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Serializer/kP", SerializerConstants.ROLLER_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Serializer/kD", SerializerConstants.ROLLER_CONSTANTS.kD);

  public Serializer(RollersIO io) {
    super("Serializer", io, SerializerConstants.ROLLER_CONSTANTS);
    this.io = io;

    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  @Override
  public void periodic() {
    super.periodic();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            io.setControlConstants(constants[0], constants[1], constants[2], constants[3]),
        kS,
        kV,
        kP,
        kD);
  }
}
