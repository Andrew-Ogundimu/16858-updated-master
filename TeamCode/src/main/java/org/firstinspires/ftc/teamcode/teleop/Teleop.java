package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="TeleOp1", group="Iterative Opmode")
public class Teleop extends OpMode {
    private boolean ac = false;
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor xrail = null;

    private static Telemetry telemetrytwo;
    private boolean change_start = false;

    private DcMotor lift = null;
    private DcMotor lift2 = null;
    public static int color = 0;

    private Servo claw = null;
    OpenCvWebcam webcam;
    Pipeline pipeline;
    private float speed = 1;
    private boolean change_x = false;
    private boolean change_y = false;
    private Servo hook = null;

    public static class Pipeline extends OpenCvPipeline
    {
        public static Point pos = new Point(0.0,0.0);
        public static Point hookPos = new Point(102,73);
        private double minConfidence = 220;
        private double min_radius = 10;
        private double circle_area = 0;
        ArrayList<Mat> RGB = new ArrayList<>(3);
        Mat threshB = new Mat();
        Mat threshR = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        ArrayList<MatOfPoint> filtered_contours = new ArrayList<>();
        private double circularity(MatOfPoint cnt) {
            float rad[] = new float[]{0.0f,0.0f};
            if (!cnt.empty()) {
                Imgproc.minEnclosingCircle(new MatOfPoint2f(cnt.toArray()), new Point(), rad);
            }
            return Math.abs(Math.PI * Math.pow(rad[0], 2) / Imgproc.contourArea(cnt) - 1);
        }

        public double getArea() {
            return circle_area;
        }
        private int min_circular(ArrayList<MatOfPoint> arr) {
            int min_ind = 0;
            for (int i=0; i<=arr.size()-1; i++) {
                if (circularity(arr.get(i))<circularity(arr.get(min_ind))) {

                    min_ind = i;
                }
            }
            return min_ind;
        }
        public Point getCone() {
            return pos;
        }
        public void moveHookPos(Point dp) {
            hookPos.x += dp.x;
            hookPos.y += dp.y;
        }
        @Override
        public Mat processFrame(Mat input) {
            RGB.clear();
            contours.clear();
            filtered_contours.clear();
            Core.split(input,RGB);
            Imgproc.threshold(RGB.get(2),threshB,minConfidence,255.0,Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(RGB.get(0),threshR,minConfidence,255.0,Imgproc.THRESH_BINARY_INV);
            if (color==0) {
                //find blue
                Imgproc.findContours(threshB, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                for (int i = 0; i < contours.size(); i++) {
                    if (Imgproc.contourArea(contours.get(i)) >= Math.PI * Math.pow(min_radius, 2)
                            && Imgproc.contourArea(contours.get(i))<=10000 && Math.abs(circularity(contours.get(i))-1)<1) {
                        filtered_contours.add(contours.get(i));
                    }
                }
                if (filtered_contours.size() > 0) {
                    MatOfPoint circ = filtered_contours.get(min_circular(filtered_contours));
                    circle_area = Imgproc.contourArea(circ);
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(circ.toArray()), pos, new float[2]);
                    Imgproc.circle(input, pos, 5, new Scalar(255, 255, 0), -1);
                    Imgproc.drawContours(input, new ArrayList<MatOfPoint>(Arrays.asList(circ)), 0, new Scalar(0, 0, 255), 5);
                    circ.release();
                } else {
                    pos.x = -1;
                    pos.y = -1;
                }
            } else if (color==1) {
                //find red
                Imgproc.findContours(threshR, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                for (int i = 0; i < contours.size(); i++) {
                    if (Imgproc.contourArea(contours.get(i)) >= Math.PI * Math.pow(min_radius, 2)
                            && Imgproc.contourArea(contours.get(i))<=10000 && Math.abs(circularity(contours.get(i))-1)<1) {
                        filtered_contours.add(contours.get(i));
                    }
                }
                if (filtered_contours.size() > 0) {
                    MatOfPoint circ = filtered_contours.get(min_circular(filtered_contours));
                    circle_area = Imgproc.contourArea(circ);
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(circ.toArray()), pos, new float[2]);
                    Imgproc.circle(input, pos, 5, new Scalar(255, 255, 0),-1);
                    Imgproc.drawContours(input, new ArrayList<MatOfPoint>(Arrays.asList(circ)), 0, new Scalar(255, 0, 0), 5);
                    circ.release();
                } else {
                    pos.x = -1;
                    pos.y = -1;
                }
            }
            Imgproc.circle(input, hookPos, 5, new Scalar(0, 255, 0), -1);
            threshB.release();
            threshR.release();
            return input;
        }
    }
    public void init() {

        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        xrail = hardwareMap.get(DcMotor.class, "xrail");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        claw = hardwareMap.get(Servo.class, "claw");
        hook = hardwareMap.get(Servo.class, "hook");

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.FORWARD);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new Pipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        telemetry.addData("Status","Initialized");
    }
    public void init_loop() {

    }
    public float[] rotate(float[] point,float degrees) {
        double d = Math.toRadians((double)degrees);
        return new float[]{(float)(Math.cos(d)*point[0]-Math.sin(d)*point[1]),(float)(Math.cos(d)*point[1]+Math.sin(d)*point[0])};
    }

    @Override
    public void loop() {

        float[] move_vec = new float[]{gamepad1.left_stick_x,gamepad1.left_stick_y};
        float[] r = rotate(move_vec,-45); //rotate the movement vector by 45 degrees
        if (ac==false) {
            bl.setPower(r[0] * speed);
            fr.setPower(r[0] * speed);
            fl.setPower(r[1] * speed);
            br.setPower(r[1] * speed);

            if (gamepad1.right_stick_x != 0) {
                bl.setPower(-gamepad1.right_stick_x * speed);
                fr.setPower(gamepad1.right_stick_x * speed);
                fl.setPower(-gamepad1.right_stick_x * speed);
                br.setPower(gamepad1.right_stick_x * speed);
            }
        }
        if (gamepad1.left_bumper){
            xrail.setPower(1.0f);
        } else if (gamepad1.left_trigger>0) {
            xrail.setPower(-1.0f);
        } else {
            xrail.setPower(0.0f);
        }
        if (gamepad1.y!=change_y && gamepad1.y) {
            if (speed<1) {
                speed += 0.1f;
            }
        } else if (gamepad1.x!=change_x && gamepad1.x) {
            if (speed>0.3) {
                speed -= 0.1f;
            }

        }
        telemetry.addData("Current Speed",speed);

        if (gamepad1.b) {
            hook.setPosition(0.8);
            telemetry.addData("Servo Position:", hook.getPosition());
        } else if (gamepad1.a){
            hook.setPosition(0);
            telemetry.addData("Servo Position:", hook.getPosition());
        }
        if (gamepad1.start && change_start == false) {
            if (color==0) {
                color = 1;
            } else if (color==1) {
                color = 0;
            }
        }
        if (color==0) {
            telemetry.addData("Alliance Set to:", "Blue");
            telemetry.addData("found Blue Cone at:",pipeline.getCone());
        } else if (color==1) {
            telemetry.addData("Alliance Set to:", "Red");
            telemetry.addData("found Red Cone at:",pipeline.getCone());
        }
        Point cone = pipeline.getCone();
        if (gamepad2.dpad_up) {
            pipeline.moveHookPos(new Point(0,-1));
        } else if (gamepad2.dpad_down) {
            pipeline.moveHookPos(new Point(0,1));
        } else if (gamepad2.dpad_left) {
            pipeline.moveHookPos(new Point(-1,0));
        } else if (gamepad2.dpad_right) {
            pipeline.moveHookPos(new Point(1,0));
        }
        double magnitude = Math.sqrt(Math.pow(pipeline.hookPos.y-cone.y,2)+Math.pow(pipeline.hookPos.x-cone.x,2));
        if (gamepad1.right_bumper) {
            ac = true;
            xrail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            xrail.setTargetPosition(-1120);
            xrail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            xrail.setPower(1.0);
        } else if (gamepad1.right_trigger>0.5) {
            ac = false;
            xrail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (ac==true) {
            if (cone.x == -1) {
                ac = false;
                xrail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                double mx = (pipeline.hookPos.x-cone.x)/magnitude;
                double my = (cone.y-pipeline.hookPos.y)/magnitude;
                bl.setPower(mx * 0.3);
                fr.setPower(mx * 0.3);
                fl.setPower(my * 0.3);
                br.setPower(my * 0.3);
            }
        }
        double coneAngle = Math.atan2(cone.y-pipeline.hookPos.y,pipeline.hookPos.x-cone.x);

        telemetry.addData("hook Pos",pipeline.hookPos);
        telemetry.addData("Cone Angle",Math.toDegrees(coneAngle));
        telemetry.addData("Cone Area",pipeline.getArea());
        change_x = gamepad1.x;
        change_y = gamepad1.y;
        change_start = gamepad1.start;
    }

}
