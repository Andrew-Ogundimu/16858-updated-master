package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auton.PIDController;
import org.firstinspires.ftc.teamcode.auton.TickService;

import java.util.ArrayList;

public class newDriveState extends State {
    private final double speed;
    private double driveSpeed_x;
    private double driveSpeed_y;

    int newLeftBackTarget;
    int newRightBackTarget;
    int newLeftFrontTarget;
    int newRightFrontTarget;
    private double distance;

    private int position_x;
    private int position_y;


    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;

    private int ticksPerTurn = 1120;
    private boolean flReached = false;
    private boolean frReached = false;
    private boolean blReached = false;
    private boolean brReached = false;
    private int threshold = 75;
    private PIDController pidDrive_x;
    private PIDController pidDrive_y;
    private int direction;
    private Telemetry telemetry;
    private double realSpeed;

    /**
     * This is the default State constructor.
     *
     * @param stateMachine The statemachine sequence to which the state belongs.
     * @param stateMachine The stateMachine sequence to which the state belongs.
     */
    public newDriveState(StateMachine stateMachine, ArrayList<DcMotor> motor, double speed, int direction, double distance) {
        super(stateMachine);
        this.distance = distance;
        this.speed = speed;
        this.direction = (direction - 45) % 360;
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

    }


    @Override
    void start() {
        //Reset the encoders back to zero for the next movement
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Bring them back to using encoders



        double distance_x = distance * Math.cos(Math.toRadians(direction));
        double distance_y = distance * Math.sin(Math.toRadians(direction));

        position_x =  TickService.inchesToTicks(distance_x);
        position_y = TickService.inchesToTicks(distance_y);

        leftBack.setTargetPosition(position_x);
        rightFront.setTargetPosition(position_x);

        leftFront.setTargetPosition(position_y);
        rightBack.setTargetPosition(position_y);


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveSpeed_x = 0.7; // this is only for initial testing
        driveSpeed_y = 0.7;

        drive(driveSpeed_x,driveSpeed_y);

    }

    @Override
    void update() {
        flReached = Math.abs(leftFront.getCurrentPosition() - position_y) < threshold;
        frReached = Math.abs(rightFront.getCurrentPosition() - position_x) < threshold;
        blReached = Math.abs(leftBack.getCurrentPosition() - position_x) < threshold;
        brReached = Math.abs(rightBack.getCurrentPosition() - position_y) < threshold;

        if (flReached) {
            leftFront.setPower(0);
        } else if (frReached) {
            rightFront.setPower(0);
        } else if (blReached) {
            leftBack.setPower(0);
        } else if (brReached) {
            rightBack.setPower(0);
        } else {
            drive(driveSpeed_x,driveSpeed_y);
        }
        if (flReached && frReached && blReached && brReached) {
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

    public void drive(double driveSpeed_x, double driveSpeed_y) {
        leftFront.setPower(driveSpeed_x);
        rightFront.setPower(driveSpeed_y);
        leftBack.setPower(driveSpeed_x);
        rightBack.setPower(driveSpeed_y);
    }

    public float[] rotate(float[] point,float degrees) {
        double d = Math.toRadians((double)degrees);
        return new float[]{(float)(Math.cos(d)*point[0]-Math.sin(d)*point[1]),(float)(Math.cos(d)*point[1]+Math.sin(d)*point[0])};
    }

}

