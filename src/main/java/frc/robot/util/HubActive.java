package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import java.util.Random;

public class HubActive {
  private static Random rand = new Random();
  private static boolean isActiveFirstRandomized = rand.nextBoolean();
  private static final int transitionPeriodSeconds = 10;
  private static final int activePeriodSeconds = 25;
  private static final int phaseShiftPeriodSeconds = 3;

  private static boolean isHubActive = true;
  private static boolean isAboutToShift = false;

  public static Optional<Boolean> isCurrentAllianceActiveFirst(boolean isFMSAttached) {
    if (!isFMSAttached) return Optional.of(isActiveFirstRandomized);

    String gameData = DriverStation.getGameSpecificMessage();

    // Transitional Period
    boolean isTransitionPhase = gameData == null || gameData.isEmpty();
    if (isTransitionPhase) return Optional.of(true);

    char allianceChar = gameData.charAt(0);
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();

    // FMS sent garbage data
    if (optionalAlliance.isEmpty()) return Optional.empty();
    Alliance myAlliance = optionalAlliance.get();

    // Every check passed, check if we are active alliance
    return Optional.of(
        (myAlliance == Alliance.Red) ? (allianceChar == 'B') : (allianceChar == 'R'));
  }

  /** Returns a pair of booleans in the form of (isHubActive, isHubAboutToChange) */
  public static Pair<Boolean, Boolean> getHubState(
      boolean fmsAttached, double teleopTimeElapsedSeconds) {

    if (teleopTimeElapsedSeconds <= transitionPeriodSeconds)
      return new Pair<Boolean, Boolean>(true, teleopTimeElapsedSeconds > phaseShiftPeriodSeconds);

    Optional<Boolean> didStartActive = isCurrentAllianceActiveFirst(fmsAttached);
    if (didStartActive.isEmpty()) return new Pair<Boolean, Boolean>(isHubActive, isAboutToShift);

    double timeElapsedExcludingTransitionPeriod =
        teleopTimeElapsedSeconds - transitionPeriodSeconds;

    if (timeElapsedExcludingTransitionPeriod > activePeriodSeconds * 4) {
      return Pair.of(true, false);
    }

    // Voodoo magic
    isHubActive =
        (timeElapsedExcludingTransitionPeriod % (activePeriodSeconds * 2) < activePeriodSeconds)
            ? didStartActive.get()
            : !didStartActive.get();

    isAboutToShift =
        (timeElapsedExcludingTransitionPeriod > 0)
            ? activePeriodSeconds - (timeElapsedExcludingTransitionPeriod % activePeriodSeconds)
                < phaseShiftPeriodSeconds
            : transitionPeriodSeconds - teleopTimeElapsedSeconds < phaseShiftPeriodSeconds;

    if (timeElapsedExcludingTransitionPeriod < 0) isHubActive = true;

    return Pair.of(isHubActive, isAboutToShift);
  }

  public static void randomizeOnTeleop() {
    isActiveFirstRandomized = rand.nextBoolean();
  }
}
