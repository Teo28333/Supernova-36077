package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake_SS;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class TeleOpDrive extends OpMode {
    private Follower follower;
    private DcMotorEx intakeMotor;
    private Intake_SS intake_ss;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "iMotor");
        intake_ss = new Intake_SS(hardwareMap, telemetry, "iMotor");
    }

    @Override
    public void loop() {
        telemetry.update();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

        follower.manual(powers);

        // relocalise button
        if (gamepad1.startWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            follower.setPose(cornerPose); // overrides our pose
        }

        if (gamepad1.right_bumper) {
            intake_ss.intakeCMD();
        } else if (gamepad1.circle) {
            intake_ss.outtakeCMD();
        } else {
            intake_ss.stop();
        }

        follower.update();

        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}