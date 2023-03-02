package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CommonVariables {

    private OpMode opMode;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime;

    public CommonVariables(OpMode opMode, Telemetry telemetry, HardwareMap hardwareMap, ElapsedTime elapsedTime) {
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.elapsedTime = elapsedTime;
    }

    public Telemetry getTelemetry() {
        return this.telemetry;
    }

    public OpMode getOpMode() {
        return this.opMode;
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }

    public ElapsedTime getElapsedTime() {
        return this.elapsedTime;
    }
}
