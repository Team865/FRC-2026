package frc.robot.subsystems.serializer;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Serializer extends Rollers {

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Serializer/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Serializer/kV", 0.0);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Serializer/kP", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Serializer/kD", 0.0);

  public Serializer(RollersIO io) {
    super("Serializer", io);
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);

    super.periodic();
  }
}
