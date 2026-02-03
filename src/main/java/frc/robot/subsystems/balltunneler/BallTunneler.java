package frc.robot.subsystems.balltunneler;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class BallTunneler extends Rollers {

  private final LoggedTunableNumber kS = new LoggedTunableNumber("BallTunneler/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("BallTunneler/kV", 0.0);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("BallTunneler/kP", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("BallTunneler/kD", 0.0);

  public BallTunneler(RollersIO io) {
    super("BallTunneler", io);
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);

    super.periodic();
  }
}
