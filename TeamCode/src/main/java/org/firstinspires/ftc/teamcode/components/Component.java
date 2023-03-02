package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommonVariables;

public abstract class Component {

    protected final OpMode opMode;
    protected final HardwareMap hardwareMap;
    protected final Telemetry telemetry;
    protected final ElapsedTime elapsedTime;

    public Component(CommonVariables commonVariables) {
        this.opMode = commonVariables.getOpMode();
        this.hardwareMap = commonVariables.getHardwareMap();
        this.telemetry = commonVariables.getTelemetry();
        this.elapsedTime = commonVariables.getElapsedTime();
    }

    public abstract void initialize();
    public abstract void stop();
    public abstract void update();

}
