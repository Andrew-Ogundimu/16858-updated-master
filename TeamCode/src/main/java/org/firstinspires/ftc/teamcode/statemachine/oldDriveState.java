package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class oldDriveState extends State {

    int newLeftBackTarget;
    int newRightBackTarget;
    int newLeftFrontTarget;
    int newRightFrontTarget;
    double distance;


    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;



    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.875;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double driveSpeed = 0.6;
    static final double TURN_SPEED = 0.5;
    private String Movement;

    /**
     * This is the default State constructor.
     *
     * @param stateMachine The statemachine sequence to which the state belongs.
     * @param stateMachine The stateMachine sequence to which the state belongs.
     */
    public oldDriveState(StateMachine stateMachine, ArrayList<DcMotor> motor, double speed, String direction, double target) {
        super(stateMachine);
        driveSpeed = speed;
        distance = target;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        Movement = direction;
        if (Movement == "turnRight" || Movement == "turnLeft") {
            distance = distance * 24/90;
        }
    }


    @Override
    void start() {
        //Reset the encoders back to zero for the next movement
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Bring them back to using encoders
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newLeftBackTarget = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRightBackTarget = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);



    }

    @Override
    void update() {
        if ((!(Movement.equals("backward"))) && (newLeftBackTarget > leftBack.getCurrentPosition() && newRightBackTarget > rightBack.getCurrentPosition() && newLeftFrontTarget > leftFront.getCurrentPosition() && newRightFrontTarget > rightFront.getCurrentPosition())) {

            if (Movement.equals("left")) {
                leftBack.setPower(driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(driveSpeed);
            } else if (Movement.equals("right")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(-driveSpeed);
            } else if (Movement.equals("backward")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(-driveSpeed);
            } else if (Movement.equals("turnRight")) {
                leftBack.setPower(-driveSpeed);
                leftFront.setPower(-driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(driveSpeed);
            } else if (Movement.equals("turnLeft")) {
                leftBack.setPower(driveSpeed);
                leftFront.setPower(driveSpeed);
                rightBack.setPower(-driveSpeed);
                rightFront.setPower(-driveSpeed);
            } else {
                leftBack.setPower(driveSpeed);
                leftFront.setPower(driveSpeed);
                rightBack.setPower(driveSpeed);
                rightFront.setPower(driveSpeed);
            }

        } else if (Movement.equals("backward") && (newLeftBackTarget < leftBack.getCurrentPosition() && newRightBackTarget < rightBack.getCurrentPosition() && newLeftFrontTarget < leftFront.getCurrentPosition() && newRightFrontTarget < rightFront.getCurrentPosition())) {
            leftBack.setPower(-driveSpeed);
            leftFront.setPower(-driveSpeed);
            rightBack.setPower(-driveSpeed);
            rightFront.setPower(-driveSpeed);


        } else {
            this.startNextState();

        }
    }


    @Override
    void stop() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        this.startTimeOut(1);

    }

    @Override
    void initialize() {
    }
}