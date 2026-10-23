package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake_SS;

public class Robot {
    private final Telemetry telemetry;
    private final Follower follower;
   // private final Intake_SS intake;
    private boolean started;
    private long lastLoopTimeNs;
    private double loopHz;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(100);
        // AUTO keeps encoder reads fresh without manual cache clearing by callers.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.create(hardwareMap);
        //intake = new Intake_SS(hardwareMap, telemetry, "iMotor");
        //intakeStop
        //intakeWrite
    }

    public void start() {
        follower.stop();
        //intakeStop
        //intakeWrite
        lastLoopTimeNs = 0L;
        loopHz = 0.0;
        started = true;
    }

    public void start(Pose startingPose) {
        resetPose(startingPose);
        start();
    }

    public void update(Gamepad gamepad) {
        if (!started) {
            return;
        }
        long now = System.nanoTime();
        if (lastLoopTimeNs != 0L && now > lastLoopTimeNs) {
            loopHz = 1.0e9 / (now - lastLoopTimeNs);
        }
        lastLoopTimeNs = now;

        follower.manual(ManualDrive.fieldCentric(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                follower.pose().heading()
        ));
        // Preserve the existing held-button controls and their priority.
        if (gamepad.right_bumper) {
            //intake
        } else if (gamepad.circle) {
            //outtake
        } else {
            //intakeStop
        }
        follower.update();
        //intake.update();

        Pose pose = follower.pose();
        telemetry.addData("Robot X", pose.x());
        telemetry.addData("Robot Y", pose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(pose.heading()));
        telemetry.addData("Loop Hz", "%.1f", loopHz);
        telemetry.update();
    }

    public void resetPose(Pose pose) {
        if (pose == null || !Double.isFinite(pose.x())
                || !Double.isFinite(pose.y()) || !Double.isFinite(pose.heading())) {
            throw new IllegalArgumentException("Robot pose must contain finite coordinates and heading.");
        }
        follower.setPose(pose);
    }

    public void stop() {
        started = false;
        try {
            follower.stop();
        } finally {
            //intake.stop();
            //intake.write();
        }
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.pose();
    }
}
