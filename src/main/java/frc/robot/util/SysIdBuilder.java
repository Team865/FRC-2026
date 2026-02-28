package frc.robot.util;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class SysIdBuilder {
  private final SysIdRoutine.Config config;
  private final SysIdRoutine.Mechanism mechanism;

  private Velocity<VoltageUnit> rampRateVoltsPerSec = null;
  private Voltage dynamicStepVoltage = null;
  private Time timeout = null;

  public SysIdBuilder(Subsystem subsystem, Consumer<Double> voltageConsumer) {
    this.config = new SysIdRoutine.Config();
    this.mechanism =
        new SysIdRoutine.Mechanism(
            voltage -> voltageConsumer.accept(voltage.in(Volts)), null, subsystem);
  }

  public SysIdBuilder withQuasistaticRampRate(double voltsPerSec) {
    this.rampRateVoltsPerSec = Volts.of(voltsPerSec).per(Second);

    return this;
  }

  public SysIdBuilder withDynamicStepVoltage(double volts) {
    this.dynamicStepVoltage = Volts.of(volts);

    return this;
  }

  public SysIdBuilder withTimeout(double timeoutSeconds) {
    this.timeout = Seconds.of(timeoutSeconds);

    return this;
  }

  public SysIdRoutine build() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            rampRateVoltsPerSec,
            dynamicStepVoltage,
            timeout,
            state ->
                Logger.recordOutput(
                    String.format("%s/SysIdTestState", mechanism.m_subsystem.getName()),
                    state.toString())),
        mechanism);
  }
}
