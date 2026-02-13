package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysIdRegister {
  private SysIdRegister() {}

  public interface SysIdTestable {
    public abstract SysIdRoutine getRoutine();
  }

  public static void register(LoggedDashboardChooser<Command> autoChooser, SysIdTestable subsystem) {
    autoChooser.addOption(String.format("%s (Dynamic Forward)", subsystem.getClass().getName()), subsystem.getRoutine().dynamic(Direction.kForward));
    autoChooser.addOption(String.format("%s (Dynamic Reverse)", subsystem.getClass().getName()), subsystem.getRoutine().dynamic(Direction.kReverse));
    autoChooser.addOption(String.format("%s (Quasistatic Forward)", subsystem.getClass().getName()), subsystem.getRoutine().quasistatic(Direction.kForward));
    autoChooser.addOption(String.format("%s (Quasistatic Reverse)", subsystem.getClass().getName()), subsystem.getRoutine().quasistatic(Direction.kReverse));
  }
}
