package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Intake_Cmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake_SS;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSS;
//turret
import java.util.List;

public class RobotAuton {

    // ── Subsystems ────────────────────────────────────────────────────────────
    public final Follower  follower;
    public final Intake_SS  intake;
    public final ShooterSS shooter;

    //public final Turret_SS  turret;
    private final List<LynxModule> controlHubs;

    // ── Commands ──────────────────────────────────────────────────────────────
    public final Intake_Cmd intakeCommands;

    // ── Alliance ──────────────────────────────────────────────────────────────
    private final boolean isBlueAlliance;

    // ── Goal position ─────────────────────────────────────────────────────────
    // ── Zero velocity fallback ────────────────────────────────────────────────
    private static final Vector ZERO_VEL = new Vector(0, 0);

    // ── State ─────────────────────────────────────────────────────────────────
    private State currentState = State.IDLE;
    private Path Path;

    private enum State {
        IDLE,
        INTAKING,
        TRANSFERRING,
        FOLLOWING,
        FOLLOWING_AND_INTAKE,
        FOLLOWING_AND_OPEN_GATE
    }

    // ── Timed action ──────────────────────────────────────────────────────────
    private final ElapsedTime actionTimer   = new ElapsedTime();
    private double             actionTimeoutMs = 0.0;
    private double             pathIntakeTimeoutMs = -1.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RobotAuton(HardwareMap hwm, Telemetry telemetry, boolean isBlueAlliance) {
        this.isBlueAlliance = isBlueAlliance;
        controlHubs = hwm.getAll(LynxModule.class);
        for (LynxModule hub : controlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.create(hwm);

        intake  = new Intake_SS(hwm, telemetry, "intake1", "intake2", "gate", "led1");
        shooter = new ShooterSS(hwm, telemetry, "shooter1", "shooter2", "hood");
        //shooter.setAutonMode(true);
      //  turret  = new TurretSS(hwm, telemetry, "turret1", "turret2");

        intakeCommands = new Intake_Cmd(intake);

        //PanelsDebug.init();
    }

    // ── Robot start ───────────────────────────────────────────────────────────
    public void start(Pose startingPose) {
        if (startingPose == null) {
            throw new IllegalArgumentException("RobotAuton startingPose cannot be null");
        }

        follower.setPose(startingPose);
    }

    // ── Main auton loop ───────────────────────────────────────────────────────
    public void update() {
        clearBulkCache();

        follower.update();

        // ── Pose & velocity ───────────────────────────────────────────────────
        Pose   pose       = follower.pose();
        double robotX     = pose.x();
        double robotY     = pose.y();
        double heading    = pose.heading();
        Vector robotVel = follower.velocity().toVector();

        if (robotVel == null) robotVel = ZERO_VEL;

        // ── Turret & shooter — always running ─────────────────────────────────
       /* turret.aimAtTargetCMD(robotX, robotY, heading, robotVel,
                aimGoalX(), aimGoalY());
        shooter.shooterSpinCMD(robotX, robotY, heading, robotVel,
                shootingGoalX(), shootingGoalY(), intakeCommands.isTransferring());*/

        // ── State machine ─────────────────────────────────────────────────────
        switch (currentState) {

            case INTAKING:
                if (intakeCommands.isFull() || actionTimer.milliseconds() >= actionTimeoutMs) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case TRANSFERRING:
                boolean timedOut = actionTimer.milliseconds() >= actionTimeoutMs;
                if (timedOut) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                } else {
                    intakeCommands.transfer();
                }
                break;

            case FOLLOWING:
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_INTAKE:
                if (pathIntakeTimeoutMs >= 0.0 && actionTimer.milliseconds() >= pathIntakeTimeoutMs) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                }

                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    pathIntakeTimeoutMs = -1.0;
                    currentState = State.IDLE;
                }
                break;

            case FOLLOWING_AND_OPEN_GATE:
                if (!follower.isBusy()) {
                    intakeCommands.idle();
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }

        // ── Command & subsystem updates ───────────────────────────────────────
        /*intakeCommands.update(
                robotX, robotY, heading, /shooter.isReady(),
                !turret.robotNeedToTurn()
        );  */

        intake.update(false);
        shooter.update(false);
       // turret.update(false);
        /*if (RobotConstants.PANELS_DEBUG_ENABLED) {
            PanelsDebug.update(follower, shooter, turret, intakeCommands.isTransferring());
        }  */
        savePose();
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    /** Intake for up to timeoutMs ms. */
    public void intakeFor(double timeoutMs) {
        intakeCommands.intake();
        actionTimeoutMs = timeoutMs;
        currentState    = State.INTAKING;
        actionTimer.reset();
    }

    /** Transfer for up to timeoutMs ms. Auto-exits when shot completes. */
    public void transferFor(double timeoutMs) {
        intakeCommands.transfer();
        actionTimeoutMs = timeoutMs;
        currentState    = State.TRANSFERRING;
        actionTimer.reset();
    }

    /** Follow a path. isBusy() returns true until the path finishes. */
    public void followPath(Path path) {
        follower.follow(path);
        currentState = State.FOLLOWING;
    }

    /** Follow a path with optional end hold. */
    public void followPath(Path path, boolean holdEnd) {
        follower.follow(path);
        currentState = State.FOLLOWING;
    }

    /** Follow a path chain. isBusy() returns true until the path finishes. */
    public void followPath() {
        follower.follow(Path);
        currentState = State.FOLLOWING;
    }

    /**
     * Follow a path while intaking simultaneously.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(Path path) {
        follower.follow(path);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }

    /**
     * Follow a path while intaking simultaneously, with optional end hold.
     * isBusy() returns true until the path finishes.
     */
    public void followPathAndIntake(Path path, boolean holdEnd) {
        follower.follow(path);
        intakeCommands.intake();
        pathIntakeTimeoutMs = -1.0;
        currentState = State.FOLLOWING_AND_INTAKE;
    }


    /**
     * Follow a path while intaking only for the first intakeTimeoutMs, with optional end hold.
     */
    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs, boolean holdEnd) {
        follower.follow(path);
        intakeCommands.intake();
        pathIntakeTimeoutMs = intakeTimeoutMs;
        currentState = State.FOLLOWING_AND_INTAKE;
        actionTimer.reset();
    }

    public void followPathAndOpenGate(Path path) {
        follower.follow(path);
        intakeCommands.openGate();
        currentState = State.FOLLOWING_AND_OPEN_GATE;
    }

    /**
     * Follow a path chain while intaking only for the first intakeTimeoutMs.
     * The path keeps running after the intake turns off.
     */
    public void followPathAndIntakeFor(Path path, double intakeTimeoutMs) {
        followPathAndIntakeFor(path, intakeTimeoutMs, true);
    }



    // ── Getters ───────────────────────────────────────────────────────────────
    public boolean isBusy()         { return currentState != State.IDLE; }
    //public boolean isShooterReady() { return shooter.isReadyForAuton();  }
    public Follower getFollower()    { return follower;                   }
    public boolean  isBlueAlliance() { return isBlueAlliance;             }
    public boolean isIntakeFull()   { return intakeCommands.isFull();     }

    public void forceIdle() {
        //follower.breakFollowing();
        intakeCommands.idle();
        currentState = State.IDLE;
        pathIntakeTimeoutMs = -1.0;
    }

    public double getPathProgressPercent() {
        if (!follower.isBusy()) {
            return 100.0;
        }

        return Math.max(0.0, Math.min(100.0, follower.completion() * 100.0));
    }

    public void savePose() {
       // PoseStorage.setCurrentPose(follower.pose());
    }

   /* private double aimGoalX() {
        return isBlueAlliance ? RobotConstants.AIM_GOAL_X_BLUE : RobotConstants.AIM_GOAL_X_RED;
    }

    private double aimGoalY() {
        return isBlueAlliance ? RobotConstants.AIM_GOAL_Y_BLUE : RobotConstants.AIM_GOAL_Y_RED;
    }

    private double shootingGoalX() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_X_BLUE : RobotConstants.SHOOTING_GOAL_X_RED;
    }

    private double shootingGoalY() {
        return isBlueAlliance ? RobotConstants.SHOOTING_GOAL_Y_BLUE : RobotConstants.SHOOTING_GOAL_Y_RED;
    } */

    private void clearBulkCache() {
        for (LynxModule hub : controlHubs) {
            hub.clearBulkCache();
        }
    }
}