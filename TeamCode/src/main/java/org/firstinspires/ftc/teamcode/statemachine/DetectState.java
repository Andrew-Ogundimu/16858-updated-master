package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.ComponentHelper;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetection;

import java.util.ArrayList;

public class DetectState extends State {

    AprilTagDetection detector;
    int measurements = 0;
    ArrayList<DcMotor> motors;
    BNO055IMU imu;

    public DetectState(StateMachine stateMachine, ArrayList<DcMotor> motors, BNO055IMU imu) {
        super(stateMachine);
        this.detector = ComponentHelper.getComponent(AprilTagDetection.class, this.commonVariables);
        this.motors = motors;
        this.imu = imu;
    }

    @Override
    void start() {
        this.detector.startStream();
        this.elapsedTime.reset();
    }

    @Override
    void update() {

        if (this.measurements >= 50 && (this.elapsedTime.milliseconds() > 1000)) {
            State[] states = {
                    new newDriveState(stateMachine, this.motors, .6, 90, 1),
                    //new oldDriveState(stateMachine, this.motors, .6, "forward", 1),
                    new DetectState(this.stateMachine, this.motors, imu),
            };
            this.insert(states);
            this.startNextState();
        } else {
            int position = this.detector.getTarget();
            if (position > 0) {
                if (position == 1) {
                    State[] states = {
                            new newDriveState(stateMachine, this.motors, .6, 0, 20.),
                            new newDriveState(stateMachine, this.motors, .6, -90, 20.),
                          //  new oldDriveState(stateMachine, this.motors, .6, "forward", 40),
                          //  new oldDriveState(stateMachine, this.motors, .6, "backward", -15),
                            //new CCWTurnByPID(stateMachine, -90, .5, motors, imu),
                         //   new oldDriveState(stateMachine, this.motors, .6, "forward", 20),
                    };
                    this.insert(states);
                } else if (position == 2) {
                    State[] states = {
                            new newDriveState(stateMachine, this.motors, .6, 270, 23),

                            //new oldDriveState(stateMachine, this.motors, .6, "forward", 23),
                    };
                    this.insert(states);
                } else if (position == 3) {
                    State[] states = {
                            new newDriveState(stateMachine, this.motors, .6, 180, 20.),
                            new newDriveState(stateMachine, this.motors, .6, -90, 20.),
                            //new oldDriveState(stateMachine, this.motors, .6, "forward", 40),
                           // new oldDriveState(stateMachine, this.motors, .6, "backward", -15),
                            new CWTurnByPID(stateMachine, 90, .5, motors, imu),
                          //  new oldDriveState(stateMachine, this.motors, .6, "forward", 20),
                    };
                    this.insert(states);
                }

                this.startNextState();
            } else {
                this.measurements += 1;
            }
        }
    }

    @Override
    void stop() {
        this.detector.stopStream();
    }

    @Override
    void initialize() {

    }
}
