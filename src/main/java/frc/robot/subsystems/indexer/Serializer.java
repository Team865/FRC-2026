package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;
import org.littletonrobotics.junction.Logger;

public class Serializer extends Rollers implements SysIdTestable {
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

  private final SysIdRoutine sysIdRoutine;
  private final Debouncer stallingDebouncer = new Debouncer(0.5, DebounceType.kRising);

  public Serializer(RollersIO io) {
    super("Serializer", io);

    io.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    sysIdRoutine =
        new SysIdBuilder(this, io::setVolts)
            .withDynamicStepVoltage(6)
            .withQuasistaticRampRate(0.6)
            .build();
  }

  public Command runSerializer() {
    return runEnd(
        () -> io.setAngularVelocity(IndexerConstants.Serializer.SPINDEXING_SPEED), () -> io.stop());
  }

  public Command startSerializer() {
    return runOnce(() -> io.setAngularVelocity(IndexerConstants.Serializer.SPINDEXING_SPEED));
  }

  public boolean isStalling() {
    return stallingDebouncer.calculate(inputs.torqueCurrentAmps > 95);
  }

  public Trigger isRunning() {
    return new Trigger(() -> inputs.angularVelocity.in(RadiansPerSecond) > 1);
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> io.setControlConstants(c[0], c[1], c[2], c[3], c[4]), kS, kV, kA, kP, kD);

    Logger.recordOutput("Serializer/PosRots", inputs.position.in(Rotations));
    Logger.recordOutput("Serializer/VelRotsPerSec", inputs.angularVelocity.in(RotationsPerSecond));

    super.periodic();
  }

  @Override
  public SysIdRoutine getRoutine() {
    return sysIdRoutine;
  }
}
