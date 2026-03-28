package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SysIdBuilder;
import frc.robot.util.SysIdRegister.SysIdTestable;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Hood extends Pivot implements SysIdTestable {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Hood/kS", ShooterConstants.Hood.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Hood/kV", ShooterConstants.Hood.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Hood/kA", ShooterConstants.Hood.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Hood/kP", ShooterConstants.Hood.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Hood/kD", ShooterConstants.Hood.SYSTEM_CONSTANTS.kD);

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Shooter/Hood/MaxVelocity", ShooterConstants.Hood.SYSTEM_CONSTANTS.maxVelocity.get());
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Shooter/Hood/MaxAcceleration",
          ShooterConstants.Hood.SYSTEM_CONSTANTS.maxAcceleration.get());

  private final Debouncer currentSenseDebouncer = new Debouncer(0.2);
  private final SysIdRoutine sysIdRoutine;

  public Hood(PivotIO io) {
    super("Shooter/Hood", io);

    io.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    io.setMotionProfile(maxVelocity.get(), maxAcceleration.get());

    io.setExtraEffort(0.1, ShooterConstants.Hood.EXTRA_GAIN_TOLERANCE);

    this.sysIdRoutine =
        new SysIdBuilder(this, io::setVolts)
            .withDynamicStepVoltage(1)
            .withQuasistaticRampRate(0.1)
            .build();
  }

  public Command trackTarget(Supplier<Angle> angleSupplier) {
    return runEnd(
        () -> {
          io.setPosition(angleSupplier.get());
        },
        () -> io.stop());
  }

  public Command manualControl(DoubleSupplier gainSupplier) {
    Command command =
        new Command() {
          private Angle currentTargetAngle;

          @Override
          public void initialize() {
            currentTargetAngle = getOrientation();
          }

          @Override
          public void execute() {
            double gain = MathUtil.applyDeadband(gainSupplier.getAsDouble(), 0.1);

            if (gain == 0.0) return;

            currentTargetAngle =
                Degrees.of(
                    MathUtil.clamp(
                        currentTargetAngle.in(Degrees)
                            + ShooterConstants.Hood.MANUAL_CONTROL_RATE * gain,
                        ShooterConstants.Hood.MIN_ANGLE_DEG,
                        ShooterConstants.Hood.MAX_ANGLE_DEG));

            io.setPosition(currentTargetAngle);
          }
        };

    command.addRequirements(this);

    return command;
  }

  public Command currentSensedRezero() {
    if (frc.robot.Constants.currentMode == frc.robot.Constants.Mode.REAL) {
      return new SequentialCommandGroup(
          setVoltage(-1),
          new WaitUntilCommand(
                  () -> currentSenseDebouncer.calculate(Math.abs(inputs.torqueCurrentAmps) > 20))
              .raceWith(new WaitCommand(2)),
          stop(),
          runOnce(() -> io.seedPosition(Rotations.zero())));
    } else {
      return runOnce(() -> {});
    }
  }

  @Override
  public void periodic() {
    // int id = hashCode();

    // LoggedTunableNumber.ifChanged(
    //     id,
    //     (constants) ->
    //         this.io.setControlConstants(
    //             constants[0], constants[1], constants[2], constants[3], constants[4]),
    //     kS,
    //     kV,
    //     kA,
    //     kP,
    //     kD);
    // LoggedTunableNumber.ifChanged(
    //     id,
    //     (constants) -> this.io.setMotionProfile(constants[0], constants[1]),
    //     maxVelocity,
    //     maxAcceleration);

    super.periodic();
  }

  @Override
  public SysIdRoutine getRoutine() {
    return this.sysIdRoutine;
  }
}
