package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {
  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("SerializerConstants/kS", SerializerConstants.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("SerializerConstants/kV", SerializerConstants.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("SerializerConstants/kP", SerializerConstants.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("SerializerConstants/kD", SerializerConstants.kD);

  public Serializer(SerializerIO io) {

    this.io = io;
    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Serializer", inputs);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            io.setControlConstants(constants[0], constants[1], constants[2], constants[3]),
        kS,
        kV,
        kP,
        kD);
  }

  @AutoLogOutput
  public Command runVoltsSerializer(double inputVolts) {
    return startEnd(() -> io.setVolts(inputVolts), () -> io.setVolts(0));
  }

  public Command runSerializeVelocityRadsPerSec(double velocityRadsPerSec) {

    return startEnd(() -> io.setVelocity(velocityRadsPerSec), () -> io.setVelocity(0));
  }
}
