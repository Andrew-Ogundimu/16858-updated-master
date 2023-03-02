package org.firstinspires.ftc.teamcode.statemachine;

public class WaitState extends State {

    private int waitTime;

    public WaitState(int waitTime, String name, StateMachine stateMachine) {
        super(stateMachine);

        this.waitTime = waitTime;
        this.setName(name);
    }

    @Override
    void start() {
        this.startTimeOut(this.waitTime);
    }

    @Override
    void update() {
        this.telemetry.addData("Message: ", "HEHEHEHE" + this.getName());
    }

    @Override
    void stop() {

    }

    @Override
    void initialize() {

    }
}
