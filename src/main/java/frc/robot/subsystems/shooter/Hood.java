package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class Hood extends Pivot {
  private final ControlSystemContext gains = ShooterConstants.Hood.getGains();
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/Hood/kS", gains.kS());
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/Hood/kV", gains.kV());
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/Hood/kA", gains.kA());
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/Hood/kP", gains.kP());
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/Hood/kD", gains.kD());

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Shooter/Hood/MaxVelocity", gains.maxVelocity().get());
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Shooter/Hood/MaxAcceleration", gains.maxAcceleration().get());

  public Hood(PivotIO io) {
    super("Shooter/Hood", io);
  }

  public Command trackTarget(Supplier<Angle> angleSupplier) {
    return runEnd(
        () -> {
          io.setPosition(angleSupplier.get().in(Radians));
        },
        () -> io.stop());
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id,
        (constants) ->
            this.io.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        kS,
        kV,
        kA,
        kP,
        kD);
    LoggedTunableNumber.ifChanged(
        id,
        (constants) -> this.io.setMotionProfile(constants[0], constants[1]),
        maxVelocity,
        maxAcceleration);

    super.periodic();
  }
}
