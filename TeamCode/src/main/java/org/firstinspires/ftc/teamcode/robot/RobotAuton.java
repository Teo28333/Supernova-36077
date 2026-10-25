
package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Intake_Cmd;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake_SS;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.Shooter_SS;

import java.util.List;

public class RobotAuton {

    public final Follower follower;
    public Pose pose;
    public Path path;

    public final Intake_SS intake;
    public final Shooter_SS shooter;
    public final Telemetry telemetry;
    public final Intake_Cmd intake_cmd;
    public final boolean isBlueAlliance;

    private final List<LynxModule> controlHubs;

    private static final Velocity ZERO_VEL = new Velocity(0, 0, 0);

    private State currentState = State.IDLE;

    public enum State {
        IDLE,
        INTAKING,
        TRANSFERRING,
        FOLLOWING,
        FOLLOWING_AND_INTAKE,
        FOLLOWING_AND_OPEN_GATE
    }

    private final ElapsedTime actionTimer = new ElapsedTime();
    private double actionTimeoutMs = 0.0;
    private double pathIntakeTimeoutMs = -1.0;

    public RobotAuton(
            HardwareMap hwm,
            Telemetry telemetry,
            boolean isBlueAlliance
    ) {
        this.isBlueAlliance = isBlueAlliance;
        this.telemetry = telemetry;

        controlHubs = hwm.getAll(LynxModule.class);

        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }

        follower = Constants.create(hwm);

        intake = new Intake_SS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new Shooter_SS(hwm, telemetry, "shooter1", "shooter2", "hood");
        intake_cmd = new Intake_Cmd(intake);
    }

    public void start(Pose startPose) {
        if (startPose == null) {
            throw new IllegalArgumentException(
                    "RobotAuton start pose cannot be null"
            );
        }

        follower.setPose(startPose);
        pose = startPose;
    }

    public void update() {
        clearBulkCache();

        follower.update();

        pose = follower.pose();

        double robotX = pose.x();
        double robotY = pose.y();
        double heading = pose.heading();

        Velocity robotVel = follower.velocity();

        if (robotVel == null) {
            robotVel = ZERO_VEL;
        }

        double angularVel = robotVel.omega;
        double robotXVel = robotVel.vx;
        double robotYVel = robotVel.vy;

        //turret + shooter always running(tkt g ps vole exactement ton commentaire de ton code)

        // shooterSpin CMD
        // X,Y Bucket pcq sinon ca vise R

        switch (currentState) {

            case INTAKING:
                if (intake_cmd.isFull()
                        || actionTimer.milliseconds() >= actionTimeoutMs) {
                    intake_cmd.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                boolean timedOut =
                        actionTimer.milliseconds() >= actionTimeoutMs;

                if (timedOut) {
                    intake_cmd.idle();
                    currentState = State.IDLE;
                } else {
                    intake_cmd.transfer();
                }
                break;

            case FOLLOWING:
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_INTAKE:
                if (pathIntakeTimeoutMs >= 0.0
                        && actionTimer.milliseconds() >= pathIntakeTimeoutMs) {
                    intake_cmd.idle();
                    pathIntakeTimeoutMs = -1.0;
                }

                if (!follower.isBusy()) {
                    intake_cmd.idle();
                    pathIntakeTimeoutMs = -1.0;
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_OPEN_GATE:
                if (!follower.isBusy()) {
                    intake_cmd.idle();
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        boolean shooterReady = shooter.isReadyForAuton();

        boolean aimedAtTarget = false;

        intake_cmd.update(shooterReady, aimedAtTarget);

        intake.update(false);
        shooter.update(false);
        //turret
        //panels debug enable
        //savepose
    }

    public void followPath(Path path) {
        follower.follow(path);
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING;
    }

    public void followPathAndIntake(Path path) {
        follower.follow(path);
        intake_cmd.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public Pose getPose() {
        return pose;
    }

    private void clearBulkCache() {
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }
}
