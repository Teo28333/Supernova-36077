package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake_SS {

    private double motorPower = 0.0;
    private boolean isIntaking = false;
    private boolean isOuttaking = false;

    private DcMotorEx motor;
    private final Telemetry telemetry;
    public Intake_SS(HardwareMap hwm, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;
        motor = hwm.get(DcMotorEx.class, motorName);
    }

    public void read() {
        telemetry.addData("is intaking", isIntaking);
        telemetry.addData("is outtaking", isOuttaking);

    }

    public void update() {
        read();
        write();
    }

    public void write() {
        motor.setPower(motorPower);

    }

    public void intakeCMD() {
        motorPower = -1.0;
        isIntaking = true;
        isOuttaking = false;
    }

    public void outtakeCMD() {
        motorPower = 1.0;
        isIntaking = false;
        isOuttaking = true;
    }

    public void stop() {
        motorPower = 0.0;
        isIntaking = false;
        isOuttaking = false;
    }
}
