package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  protected final RollersIO io;
  protected final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  protected final RollersConstants constants;

  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kD;

  public Rollers(String name, RollersIO io, RollersConstants constants) {
    this.io = io;
    this.constants = constants;

    kS = new LoggedTunableNumber(name + "/kS", constants.kS);
    kV = new LoggedTunableNumber(name + "/kV", constants.kV);
    kP = new LoggedTunableNumber(name + "/kP", constants.kP);
    kD = new LoggedTunableNumber(name + "/kD", constants.kD);

    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), c -> io.setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);
  }

  @AutoLogOutput
  public Command runVolts(double volts) {
    return startEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runVelocity(double velocityRadPerSec) {
    return startEnd(() -> io.setVelocity(velocityRadPerSec), () -> io.setVelocity(0));
  }

  public Command runLinearVelocity(double metersPerSec) {
    double angularRadPerSec = metersPerSec / constants.rollerRadiusMeters * constants.gearRatio;
    return runVelocity(angularRadPerSec);
  }

  public RollersIO getIO() {
    return io;
  }
}
