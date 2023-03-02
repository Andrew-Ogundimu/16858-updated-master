package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommonVariables;

/**
 * This is the state, the logical foundational unit of the State Machine architecture.
 * This is an abstract class, meant to be extended and to have its logic defined by actual states.
 * @author Lawson
 * @version 1.0
 * @since November 20, 2021
 */
public abstract class State {

    protected final StateMachine stateMachine;

    protected final CommonVariables commonVariables;
    protected final Telemetry telemetry;
    protected final HardwareMap hardwareMap;
    protected final ElapsedTime elapsedTime;
    protected final OpMode opMode;

    private double startTime;
    private double timeoutTime = 0;
    private State nextState = null;
    private boolean running = false;

    private String name = this.getClass().getSimpleName();


    // CONSTRUCTORS

    /**
     * This is the default State constructor.
     * @param stateMachine The statemachine sequence to which the state belongs.
     */
    public State(StateMachine stateMachine) {
        this.stateMachine = stateMachine;

        this.commonVariables = this.stateMachine.commonVariables;
        this.telemetry = this.stateMachine.telemetry;
        this.hardwareMap = this.stateMachine.hardwareMap;
        this.elapsedTime = this.stateMachine.elapsedTime;
        this.opMode = this.stateMachine.opMode;
    }

    // METHODS

    /**
     * Converts this class to a string using it's name.
     * @return The name of the state (by default, it's the class name).
     */
    public String toString(){
        return this.name;
    }

    /**
     * Returns the name of the state. By default it is the class name.
     * Logically equivalent to toString().
     * @return The name of the state.
     */
    public String getName() {
        return this.toString();
    }

    /**
     * Sets the name of the state to the specified name.
     * @param newName The new name being assigned to this state instance.
     */
    public void setName(String newName) {
        this.name = newName;
    }

    /**
     * Returns the next state after this one.
     * @return The state instance of the following state.
     */
    public State getNextState() {
        return this.nextState;
    }

    /**
     * Sets the next state after this one to the specified state.
     * @param nextState The state which will follow this one.
     */
    public void setNextState(State nextState) {
        this.nextState = nextState;
    }

    /**
     * Returns a boolean indicating if the state is running or not.
     * @return A boolean indicating if the state is running or not.
     */
    public boolean isRunning() {
        return this.running;
    }

    /**
     * Inserts a state to run between this one and what would have come after it.
     * @param newState The state being inserted.
     */
    public void insert(State newState) {
        if (this.nextState != null) {
            newState.setNextState(this.nextState);
        }

        this.nextState = newState;
    }

    public void insert(State[] newStates) {
        State oldNextState = this.nextState;
        State lastState = this;

        for (State stateToAdd : newStates) {
            lastState.setNextState(stateToAdd);

            stateToAdd.initialize();

            lastState = stateToAdd;
        }

        lastState.setNextState(oldNextState);
    }

    /**
     * Attempts to have the thread wait for the specified amount of time.
     * @param milliseconds The time that the thread will sleep for.
     */
    protected void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Stops this state and stops the next one.
     */
    protected void startNextState() {
        this._stop();

        if (this.nextState != null) {
            this.nextState._start();
        }
    }

    /**
     * Abstract start method that encapsulates start logic.
     */
    abstract void start();

    /**
     * Internal state method that is used to trigger start() and handles behind the scenes starting logic.
     */
    protected void _start() {
        this.elapsedTime.reset();
        this.startTime = this.elapsedTime.milliseconds();

        this.running = true;

        this.start();
    }

    /**
     * Abstract update method that encapsulates update logic.
     */
    abstract void update();

    /**
     * Internal update method that is used to trigger update() and handles behind the scenes updating logic.
     */
    protected void _update() {
        if (this.timeoutTime > 0) { // If a timeout time has been set
            double currentTime = this.elapsedTime.milliseconds(); // Get the current time
            if (currentTime > this.timeoutTime) { // Check if current time exceeds the timeout time.
                this.startNextState();
                return;
            }
        }

        this.update();
    }

    /**
     * Abstract stop method that encapsulates stop logic.
     */
    abstract void stop();

    /**
     * Internal stop method that is used to trigger stop() and handles behind the scenes stopiing logic.
     */
    protected void _stop() {
        this.running = false;
        this.stop();
    }

    /**
     * Abstract method to handle initialization logic.
     */
    abstract void initialize();

    /**
     * Starts a timeout for the given amount of time. After this time limit has been reached, the state will end.
     * @param timeoutLength The amount of time, in seconds, to wait before ending the state.
     */
    protected void startTimeOut(int timeoutLength) {
        this.timeoutTime = this.elapsedTime.milliseconds() + (1000 * timeoutLength);
    }

    /**
     * Stops a currently running timeout.
     */
    protected void stopTimeOut() {
        this.timeoutTime = 0;
    }
}
