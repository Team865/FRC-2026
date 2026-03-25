package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class PitCheckCommand<Goal> extends Command {
  private final Consumer<Goal> driver;
  private final BooleanSupplier checkGoalSupplier;
  private final double timeoutSeconds;
  private final Goal[] goals;

  private int currentGoalIndex = 0;
  private long goalStartTimeMillis = 0;

  public PitCheckCommand(
      String name,
      Consumer<Goal> driver,
      BooleanSupplier checkGoalSupplier,
      double timeoutSeconds,
      Goal[] goals,
      Subsystem... subsystems) {
    if (goals.length == 0) throw new IllegalArgumentException("There must be at least 1 goal.");

    this.setName(name);
    this.driver = driver;
    this.checkGoalSupplier = checkGoalSupplier;
    this.timeoutSeconds = timeoutSeconds;
    this.goals = goals;

    addRequirements(subsystems);
  }

  @Override
  public void initialize() {
    currentGoalIndex = 0;
    goalStartTimeMillis = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    if (currentGoalIndex >= goals.length) return;

    driver.accept(goals[currentGoalIndex]);

    boolean goalReached = checkGoalSupplier.getAsBoolean();

    if (goalReached
        || ((double) (System.currentTimeMillis() - goalStartTimeMillis) / 1000.0)
            > timeoutSeconds) {
      if (goalReached) {
        System.out.printf("Goal %s succeeded for pit check %s", currentGoalIndex, getName());
      } else {
        System.err.printf("Goal %s failed for pit check %s", currentGoalIndex, getName());
      }

      currentGoalIndex++;
      goalStartTimeMillis = System.currentTimeMillis();
    }
  }

  @Override
  public boolean isFinished() {
    return currentGoalIndex == goals.length;
  }
}
