package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.components.ComponentHelper;
import org.firstinspires.ftc.teamcode.statemachine.DetectState;
import org.firstinspires.ftc.teamcode.statemachine.State;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name="Auton")
public class Auton extends OpMode {
    // Declare OpMode members.
    private CommonVariables commonVariables;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private StateMachine stateMachine;
    private Component[] opModeComponents;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    BNO055IMU imu;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        leftFront  = hardwareMap.get(DcMotor.class, "fl"); //0
        rightFront = hardwareMap.get(DcMotor.class, "fr"); //1
        leftBack  = hardwareMap.get(DcMotor.class, "bl"); //2
        rightBack = hardwareMap.get(DcMotor.class, "br"); //3 ?

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE); //leftFront
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");

        this.commonVariables = new CommonVariables(
                this,
                this.telemetry,
                this.hardwareMap,
                this.elapsedTime
        );

        this.stateMachine = new StateMachine(this.commonVariables);

        State[] states = {

                new DetectState(this.stateMachine, motors, imu)


        };

        this.stateMachine.feed(states);

        Component[] components = {
                ComponentHelper.getComponent(AprilTagDetection.class, commonVariables)
        };
        this.opModeComponents = components;

        for (Component component : components) {
            component.initialize();
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        this.telemetry.addData("Target", ComponentHelper.getComponent(AprilTagDetection.class, commonVariables).getTarget());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        elapsedTime.reset();
        // START MANAGER
        this.stateMachine.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        this.stateMachine.update();
        telemetry.addData("LeftBack", leftBack.getCurrentPosition());
        telemetry.addData("LeftFront", leftFront.getCurrentPosition());
        telemetry.addData("RightBack", rightBack.getCurrentPosition());
        telemetry.addData("RightFront", rightFront.getCurrentPosition());
        this.telemetry.addData("Target:", ComponentHelper.getComponent(AprilTagDetection.class, commonVariables).getTarget());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        this.stateMachine.stop();
        this.stateMachine = null;

        for (Component component : this.opModeComponents) {
            component.stop();
        }

    }
}
