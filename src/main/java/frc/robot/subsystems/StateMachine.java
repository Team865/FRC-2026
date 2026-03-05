package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class StateMachine<State extends Enum<State>> extends SubsystemBase {
  public static enum RequestCode {
    /** State change request succeeded */
    SUCCESS,
    /**
     * This state doesn't have requirements, meaning it can't be requested. This likely means it is
     * a state used internally, rather a state that is meant to be requested externally.
     */
    CANNOT_REQUEST,
    /** The requirements to switch to the state aren't met */
    REQUIREMENTS_FAILED,
    /** The current state is already set to this */
    ALREADY_SET
  }

  /** A trigger for when each state is entered */
  public final Map<State, Trigger> stateTriggers;

  /** The requirements to move to this state */
  public final Map<State, BooleanSupplier> stateRequirements = new HashMap<>();

  private State currentState;

  /** Default state requirement meaning the state can always be switched to */
  public static final BooleanSupplier STATE_ALWAYS_AVAILABLE = () -> true;

  public StateMachine(State initialState) {
    this.currentState = initialState;

    Map<State, Trigger> modifiableStateTriggers = new HashMap<>();

    // Initialize state triggers
    for (State state : initialState.getDeclaringClass().getEnumConstants()) {
      modifiableStateTriggers.put(
          state, new Trigger(() -> this.currentState.equals(state) && DriverStation.isEnabled()));
    }

    this.stateTriggers = Collections.unmodifiableMap(modifiableStateTriggers);
  }

  /**
   * @return The current state of the state machine
   */
  public State getState() {
    return currentState;
  }

  /**
   * Forces a certain state
   *
   * @param newState The new state the state machine should be in
   */
  public void forceState(State newState) {
    this.currentState = newState;
  }

  /** Command wrapper for {@link #forceState(Enum)} */
  public Command forceStateCommand(State newState) {
    return runOnce(() -> forceState(newState));
  }

  /**
   * @return Whether the state machine is currently in said state
   */
  public boolean isInState(State stateToCompareTo) {
    return this.currentState.equals(stateToCompareTo);
  }

  /**
   * Request to switch to a new state and returns a RequestCode representing whether it was
   * successful
   *
   * @param targetState The requested state
   * @return A {@link RequestCode} representing the request result
   */
  public RequestCode requestState(State targetState) {
    // If the state is already set
    if (currentState.equals(targetState)) return RequestCode.ALREADY_SET;
    // If no state requirement exists
    if (!stateRequirements.containsKey(targetState)) return RequestCode.CANNOT_REQUEST;
    // If the state requirement isn't met
    if (!stateRequirements.get(targetState).getAsBoolean()) return RequestCode.REQUIREMENTS_FAILED;

    forceState(targetState);
    return RequestCode.SUCCESS;
  }

  /** Command wrapper for {@link #requestState(Enum)} */
  public Command requestStateCommand(State targetState) {
    return runOnce(() -> requestState(targetState));
  }

  /**
   * Runs {@link #requestState(Enum)} until the state machine is in said state or command is
   * interrupted.
   */
  public Command runRequestStateCommand(State targetState) {
    return run(() -> requestState(targetState)).until(() -> isInState(targetState));
  }
}
