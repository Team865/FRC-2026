package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FullSubsystem extends SubsystemBase {
  public abstract Command stop();
}
