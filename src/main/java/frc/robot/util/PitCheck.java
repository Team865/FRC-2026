package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.stream.Stream;

public class PitCheck<Goal> extends Command {
  private final Consumer<Goal> driver;
  private final Predicate<Goal> checkGoalSupplier;
  private final double timeoutSeconds;
  private final Goal[] goals;
  private final Debouncer goalDebouncer;
  private final Command[] stopCommands;

  private static Superstructure superstructure;
  private int currentGoalIndex = 0;
  private long goalStartTimeMillis = 0;

  public static <Goal> Command createCommand(
      String name,
      Consumer<Goal> driver,
      Predicate<Goal> checkGoalSupplier,
      double goalDebounceSeconds,
      double timeoutSeconds,
      Goal[] goals,
      FullSubsystem... subsystems) {
    return superstructure
        .startManualOverride()
        .andThen(
            new PitCheck<>(
                    name,
                    driver,
                    checkGoalSupplier,
                    goalDebounceSeconds,
                    timeoutSeconds,
                    goals,
                    subsystems)
                .ignoringDisable(true));
  }

  private PitCheck(
      String name,
      Consumer<Goal> driver,
      Predicate<Goal> checkGoalSupplier,
      double goalDebounceSeconds,
      double timeoutSeconds,
      Goal[] goals,
      FullSubsystem... subsystems) {
    if (goals.length == 0) throw new IllegalArgumentException("There must be at least 1 goal.");

    this.setName(name);
    this.driver = driver;
    this.checkGoalSupplier = checkGoalSupplier;
    this.timeoutSeconds = timeoutSeconds;
    this.goals = goals;
    this.goalDebouncer = new Debouncer(goalDebounceSeconds, DebounceType.kRising);

    addRequirements(subsystems);
    this.stopCommands =
        Stream.of(subsystems)
            .map(subsystem -> subsystem.stop().ignoringDisable(true))
            .toArray(Command[]::new);
  }

  public static void registerSuperstructure(Superstructure superstructure) {
    PitCheck.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    currentGoalIndex = 0;
    goalStartTimeMillis = System.currentTimeMillis();
    System.out.println(getName() + " started.");
  }

  @Override
  public void execute() {
    if (currentGoalIndex >= goals.length) return;

    Goal currentGoal = goals[currentGoalIndex];

    driver.accept(currentGoal);

    boolean goalReached = goalDebouncer.calculate(checkGoalSupplier.test(currentGoal));
    double wallTimeSeconds = (double) (System.currentTimeMillis() - goalStartTimeMillis) / 1000.0;

    if (goalReached || wallTimeSeconds > timeoutSeconds) {
      if (goalReached) {
        System.out.printf(
            "Goal %s succeeded for pit check %s in %.2f seconds.\n",
            currentGoalIndex, getName(), wallTimeSeconds);
      } else {
        System.err.printf(
            "Goal %s failed for pit check %s in %.2f seconds.\n",
            currentGoalIndex, getName(), wallTimeSeconds);
      }

      currentGoalIndex++;
      goalStartTimeMillis = System.currentTimeMillis();
      goalDebouncer.calculate(false); // Reset Debouncer
    }
  }

  @Override
  public boolean isFinished() {
    return currentGoalIndex == goals.length;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) System.out.println(getName() + " ended.");
    else System.out.println(getName() + " got interrupted.");

    CommandScheduler.getInstance().schedule(stopCommands);
  }
}
