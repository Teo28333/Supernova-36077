package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class ShooterTuner extends OpMode {
    private Robot robot;
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
    }
    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        telemetry.update();
        robot.update(gamepad1);
    }
}