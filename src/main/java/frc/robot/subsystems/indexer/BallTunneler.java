package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;
import org.littletonrobotics.junction.Logger;

public class BallTunneler extends Rollers implements SysIdTestable {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("BallTunneler/kS", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("BallTunneler/kV", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("BallTunneler/kA", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("BallTunneler/kP", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("BallTunneler/kD", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kD);

  private final SysIdRoutine sysIdRoutine;

  public BallTunneler(RollersIO io) {
    super("BallTunneler", io);

    io.setControlConstants(kV.get(), kA.get(), kS.get(), kP.get(), kD.get());
    sysIdRoutine = new SysIdBuilder(this, io::setVolts).withDynamicStepVoltage(6.0).build();
  }

  public Command runTunneler() {
    return runEnd(
        () ->
            io.setAngularVelocity(
                IndexerConstants.BallTunneler.TUNNELING_SPEED.in(RadiansPerSecond)),
        () -> io.stop());
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3], c[4]), kV, kA, kS, kP, kD);
    Logger.recordOutput(
        "BallTunneler/VelocityRotsPerSec", Units.radiansToRotations(inputs.velocityRadsPerSec));
    Logger.recordOutput("BallTunneler/PositionRots", Units.radiansToRotations(inputs.positionRads));
    super.periodic();
  }

  @Override
  public SysIdRoutine getRoutine() {
    return sysIdRoutine;
  }
}
