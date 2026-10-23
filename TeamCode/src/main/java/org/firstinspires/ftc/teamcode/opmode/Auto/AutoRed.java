package org.firstinspires.ftc.teamcode.opmode.Auto;

import static com.pedropathing.api.Paths.*;

import android.widget.Switch;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "AutoRed", group = "Examples")
public class AutoRed {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(32.5, 9, 180);
    private final Pose fleur1 = poseFactory.of(10, 47, 180);
    private final Pose fleur1Control1 = poseFactory.of(35, 44, 0);
    private final Pose flaur2ShotStart = poseFactory.of(10, 47, 180);
    private final Pose flaur2Shot = poseFactory.of(47, 132, 90);
    private final Pose flaur2ShotControl1 = poseFactory.of(35, 82, 0);
    private final Pose intakePrepStart = poseFactory.of(47, 132, 90);
    private final Pose intakePrep = poseFactory.of(32, 9, 180);
    private final Pose intakePrepControl1 = poseFactory.of(82, 45, 0);
    private final Pose intakeStart = poseFactory.of(32, 9, 180);
    private final Pose intake = poseFactory.of(9, 9, 180);
    private final Pose parkShootStart = poseFactory.of(9, 9, 180);
    private final Pose parkShoot = poseFactory.of(9, 103, 180);
    private final Pose parkShootControl1 = poseFactory.of(51, 81, 0);





    public Path fleur1() {
        return Paths.curve(start, fleur1Control1, fleur1).linear(start, fleur1);
    }

    public Path fleur2Shot() {
        return Paths.curve(flaur2ShotStart, flaur2ShotControl1, flaur2Shot).linear(flaur2ShotStart, flaur2Shot);
    }

    public Path intakePrep() {
        return Paths.curve(intakePrepStart, intakePrepControl1, intakePrep).linear(intakePrepStart, intakePrep);
    }

    public Path intake() {
        return Paths.line(intakeStart, intake).linear(intakeStart, intake);
    }

    public Path parkShoot() {
        return Paths.curve(parkShootStart, parkShootControl1, parkShoot).linear(parkShootStart, parkShoot);
    }

}

