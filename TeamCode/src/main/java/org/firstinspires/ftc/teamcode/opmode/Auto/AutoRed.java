
package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotAuton;
import org.firstinspires.ftc.teamcode.opmode.Auto.Path.AutoPath;

@Autonomous(name = "Auto Red")
public class AutoRed extends OpMode {

    private Follower follower;
    private RobotAuton robot;
    private AutoPath path;

    private enum AutoStep {
        FLEUR1,
        FLEUR2_SHOT,
        INTAKE_PREP,
        INTAKE,
        PARK_SHOOT,
        DONE
    }

    private AutoStep step = AutoStep.FLEUR1;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        robot = new RobotAuton(hardwareMap, telemetry, false);
        path = new AutoPath();
        robot.start(path.start);
        telemetry.addLine("Auto Red Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        step = AutoStep.FLEUR1;
        robot.followPath(path.fleur1());
    }

    @Override
    public void loop() {
        robot.update();

        if (step != AutoStep.DONE && !robot.isBusy()) {
            startNextStep();
        }

        telemetry.addData("pathState", step);
        telemetry.addData("Robot occupé", robot.isBusy());
        telemetry.addData("X", robot.getPose().x());
        telemetry.addData("Y", robot.getPose().y());
        telemetry.update();
    }

    private void startNextStep() {
        switch (step) {

            case FLEUR1:
                step = AutoStep.FLEUR2_SHOT;
                robot.followPath(path.fleur2Shot());
                break;

            case FLEUR2_SHOT:
                step = AutoStep.INTAKE_PREP;
                robot.followPath(path.intakePrep());
                break;

            case INTAKE_PREP:
                step = AutoStep.INTAKE;
                robot.followPath(path.intake());
                break;

            case INTAKE:
                step = AutoStep.PARK_SHOOT;
                robot.followPathAndIntake(path.parkShoot());
                break;

            case PARK_SHOOT:
                step = AutoStep.DONE;
                break;

            case DONE:
                break;
        }
    }
}
