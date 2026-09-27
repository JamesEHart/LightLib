package io.github.jamesehart.lightutil.commands;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A simple state machine over an enum, for subsystems or a superstructure.
 *
 * <pre>{@code
 * enum State { IDLE, INTAKING, HOLDING }
 *
 * StateMachine<State> intake = new StateMachine<>(State.IDLE)
 *     .onEnter(State.INTAKING, () -> roller.setVoltage(8))
 *     .onExit(State.INTAKING, () -> roller.setVoltage(0))
 *     .transition(State.INTAKING, State.HOLDING, hasPiece)
 *     .transition(State.HOLDING, State.IDLE, () -> !hasPiece.getAsBoolean());
 *
 * // in periodic():
 * intake.update();
 * LightLogger.logString("Intake/State", intake.getState().name());
 * }</pre>
 *
 * @param <E> the state enum
 */
public final class StateMachine<E extends Enum<E>> {
    private record Transition<E>(E to, BooleanSupplier condition) {}

    private final Map<E, Runnable> enterActions;
    private final Map<E, Runnable> exitActions;
    private final Map<E, Runnable> whileActions;
    private final Map<E, List<Transition<E>>> transitions;
    private E state;
    private E requested;
    private double enteredAt = Double.NaN;

    /**
     * Creates a state machine. The starting state's enter action runs on the first {@link #update()}.
     *
     * @param initial the starting state
     */
    public StateMachine(E initial) {
        Class<E> type = initial.getDeclaringClass();
        enterActions = new EnumMap<>(type);
        exitActions = new EnumMap<>(type);
        whileActions = new EnumMap<>(type);
        transitions = new EnumMap<>(type);
        requested = initial;
    }

    /** Runs {@code action} once each time the machine enters {@code state}. */
    public StateMachine<E> onEnter(E state, Runnable action) {
        enterActions.put(state, action);
        return this;
    }

    /** Runs {@code action} once each time the machine leaves {@code state}. */
    public StateMachine<E> onExit(E state, Runnable action) {
        exitActions.put(state, action);
        return this;
    }

    /** Runs {@code action} every {@link #update()} while in {@code state}. */
    public StateMachine<E> whileIn(E state, Runnable action) {
        whileActions.put(state, action);
        return this;
    }

    /**
     * Moves from {@code from} to {@code to} when {@code condition} is true. Transitions out of a
     * state are checked in the order they were added, and at most one happens per update.
     */
    public StateMachine<E> transition(E from, E to, BooleanSupplier condition) {
        transitions.computeIfAbsent(from, k -> new ArrayList<>()).add(new Transition<>(to, condition));
        return this;
    }

    /** Requests a state change, e.g. from a button. It happens on the next {@link #update()}. */
    public void setState(E newState) {
        requested = newState;
    }

    /** Checks transitions and runs the state's actions. Call once per loop, e.g. in periodic(). */
    public void update() {
        if (requested == null && state != null) {
            for (Transition<E> t : transitions.getOrDefault(state, List.of())) {
                if (t.condition().getAsBoolean()) {
                    requested = t.to();
                    break;
                }
            }
        }
        if (requested != null) {
            E next = requested;
            requested = null;
            if (next != state) {
                if (state != null) run(exitActions.get(state));
                state = next;
                enteredAt = Timer.getFPGATimestamp();
                run(enterActions.get(state));
            }
        }
        run(whileActions.get(state));
    }

    private static void run(Runnable action) {
        if (action != null) action.run();
    }

    /** The current state, or null before the first {@link #update()}. */
    public E getState() {
        return state;
    }

    /** True if the machine is in {@code s}. */
    public boolean is(E s) {
        return state == s;
    }

    /** A trigger that's true while the machine is in {@code s}. */
    public Trigger in(E s) {
        return new Trigger(() -> state == s);
    }

    /** Seconds since the current state was entered. */
    public double timeInState() {
        return Double.isNaN(enteredAt) ? 0 : Timer.getFPGATimestamp() - enteredAt;
    }
}
