package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake_SS {

    private DcMotorEx motor;
    private final Telemetry telemetry;
    public Intake_SS(HardwareMap hwm, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;
        motor = hwm.get(DcMotorEx.class, motorName);
    }

    public void read() {

    }

    public void update() {

    }

    public void write() {

    }
}
