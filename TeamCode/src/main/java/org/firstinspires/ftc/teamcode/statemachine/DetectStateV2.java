package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.ComponentHelper;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetection;

import java.util.ArrayList;

public class DetectStateV2 extends State {

    AprilTagDetection detector;
    int measurements = 0;
    ArrayList<DcMotor> motors;
    BNO055IMU imu;

    public DetectStateV2(StateMachine stateMachine, ArrayList<DcMotor> motors, BNO055IMU imu) {
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
                //    new DriveState(stateMachine, this.motors, .6, "forward", 1),
                    new DetectStateV2(this.stateMachine, this.motors, imu),
            };
            this.insert(states);
            this.startNextState();
        } else {
            int position = this.detector.getTarget();
            if (position > 0) {
                if (position == 1) {
                    State[] states = {
                   //         new DriveState(stateMachine, this.motors, .6, "forward", 45),
                     //       new CCWTurnByPID(stateMachine, -90, .5, motors, imu),
                       //     new DriveState(stateMachine, this.motors, .6, "forward", 20),
                    };
                    this.insert(states);
                } else if (position == 2) {
                    State[] states = {
                //            new DriveState(stateMachine, this.motors, .6, "forward", 45),
                    };
                    this.insert(states);
                } else if (position == 3) {
                    State[] states = {
                   //         new DriveState(stateMachine, this.motors, .6, "forward", 45),
                            new CWTurnByPID(stateMachine, 90, .5, motors, imu),
                    //        new DriveState(stateMachine, this.motors, .6, "forward", 20),
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
