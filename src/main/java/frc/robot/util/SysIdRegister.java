package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SysIdRegister {
  private SysIdRegister() {}

  public interface SysIdTestable {
    public abstract SysIdRoutine getRoutine();
  }

  public static void register(
      LoggedDashboardChooser<Command> autoChooser, SysIdTestable subsystem, String subsystemName) {
    autoChooser.addOption(
        String.format("%s SysId (Dynamic Forward)", subsystemName),
        subsystem.getRoutine().dynamic(Direction.kForward));
    autoChooser.addOption(
        String.format("%s SysId (Dynamic Reverse)", subsystemName),
        subsystem.getRoutine().dynamic(Direction.kReverse));
    autoChooser.addOption(
        String.format("%s SysId (Quasistatic Forward)", subsystemName),
        subsystem.getRoutine().quasistatic(Direction.kForward));
    autoChooser.addOption(
        String.format("%s SysId (Quasistatic Reverse)", subsystemName),
        subsystem.getRoutine().quasistatic(Direction.kReverse));
  }
}
