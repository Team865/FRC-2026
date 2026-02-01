package frc.robot.subsystems.balltunneler;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class BallTunneler extends Rollers {

  private final RollersIO io;

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("BallTunneler/kS", BallTunnelerConstants.ROLLER_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("BallTunneler/kV", BallTunnelerConstants.ROLLER_CONSTANTS.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("BallTunneler/kP", BallTunnelerConstants.ROLLER_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("BallTunneler/kD", BallTunnelerConstants.ROLLER_CONSTANTS.kD);

  public BallTunneler(RollersIO io) {
    super("BallTunneler", io, BallTunnelerConstants.ROLLER_CONSTANTS);
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
