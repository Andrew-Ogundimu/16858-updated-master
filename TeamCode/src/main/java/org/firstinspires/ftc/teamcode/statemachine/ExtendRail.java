package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class ExtendRail extends State {

    private final double power = .5;
    private int inches;
    private int targetTicks;
    private DcMotor railMotor;
   // private Servo swinggy;
   // private Servo armServo;

    /**
     * This is the default State constructor.
     *
     * @param stateMachine The statemachine sequence to which the state belongs.
     */
    public ExtendRail(StateMachine stateMachine, int inches) {
        super(stateMachine);

        this.inches = inches;

    }

    void changeTarget(int newInches) {
        this.inches = newInches;

    }

    @Override
    void start() {
        this.railMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.railMotor.setTargetPosition(this.targetTicks);
        this.railMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.railMotor.setPower(this.power);
    }

    @Override
    void update() {
        if (this.railMotor.getCurrentPosition() < this.railMotor.getTargetPosition() && elapsedTime.seconds() < 3) {
            this.railMotor.setPower(this.power);
            // this.swinggy.setPosition(1);
           // this.armServo.setPosition(0);
        }
        else {

            this.startNextState();
        }
    }

    @Override
    void stop() {

        this.railMotor.setPower(0);
       // this.swinggy.setPosition(0.5);
      //  this.armServo.setPosition(0.5);


    }

    @Override
    void initialize() {
        this.railMotor = this.hardwareMap.get(DcMotor.class, "xr");
        // this.armServo = this.hardwareMap.get(Servo.class, "cs");
       // this.swinggy = this.hardwareMap.get(Servo.class, "ts");
    }
}
