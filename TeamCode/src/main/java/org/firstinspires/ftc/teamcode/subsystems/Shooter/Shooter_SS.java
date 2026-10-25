
package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import org.firstinspires.ftc.teamcode.math.ShooterEquation;

import static org.firstinspires.ftc.teamcode.constants.ShooterConstants.*;

import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Shooter_SS {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private Servo hood;
    private Servo led;

    private Telemetry telemetry;

    private double motorPow = 0.0;
    private double hoodPos = 0.0;

    private double currentVelRPM = 0.0;
    private double targetRPM = 0.0;
    private double correctedTargetRPM = 0.0;
    private double rpmDrop = 0.0;
    private double hoodDrop = 0.0;
    private double currentDistance = 0.0;

    private final ShooterEquation shooterEquation =
            new ShooterEquation();

    private boolean tuningMode = false;
    private boolean autonMode = false;

    public Shooter_SS(
            HardwareMap hwm,
            Telemetry telemetry,
            String shooterMotor1,
            String shooterMotor2,
            String hoodServo
    ) {
        this.telemetry = telemetry;

        motor1 = hwm.get(DcMotorEx.class, shooterMotor1);
        motor2 = hwm.get(DcMotorEx.class, shooterMotor2);

        //inverser si pas frontspin
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );
        motor2.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        hood = hwm.get(Servo.class, hoodServo);
        led = hwm.get(Servo.class, "led2");
    }

    public void read() {
        currentVelRPM = Math.abs(
                tpsToRPM(motor1.getVelocity())
        );
    }

    public void update() {
        read();
        write();
        telemetry();
    }

    public void update(boolean showTelemetry) {
        read();
        write();

        if (showTelemetry) {
            telemetry();
        }
    }

    public void write() {
        motor1.setPower(motorPow);
        motor2.setPower(motorPow);

        hood.setPosition(hoodPos);
    }

    public void telemetry() {
        telemetry.addData("Shooter target (RPM)", targetRPM);
        telemetry.addData(
                "Shooter corrected target (RPM)",
                correctedTargetRPM
        );
        telemetry.addData(
                "Shooter current (RPM)",
                currentVelRPM
        );
        telemetry.addData(
                "Shooter error (RPM)",
                targetRPM - currentVelRPM
        );
        telemetry.addData("Shooter RPM drop", rpmDrop);
        telemetry.addData("Shooter power", motorPow);
        telemetry.addData("Hood recovery drop", hoodDrop);
        telemetry.addData("Hood position", hoodPos);
        telemetry.addData("Distance (in)", currentDistance);
        telemetry.addData("Tuning mode", tuningMode);
    }

    public void shooterSpinCMD(
            double robotX,
            double robotY,
            double robotHeading,
            Velocity robotVel,
            double goalX,
            double goalY
    ) {
        shooterSpinCMD(
                robotX,
                robotY,
                robotHeading,
                robotVel,
                goalX,
                goalY,
                false
        );
    }

    public void shooterSpinCMD(
            double robotX,
            double robotY,
            double robotHeading,
            Velocity robotVel,
            double goalX,
            double goalY,
            boolean transferActive
    ) {
        if (robotVel == null) {
            robotVel = new Velocity(0, 0,0);
        }

        double distance = Math.hypot(
                goalX - robotX,
                goalY - robotY
        );

        double dt = shooterEquation.getAirTime(distance);

        double adjGoalX = goalX - robotVel.vx * dt;
        double adjGoalY = goalY - robotVel.vy * dt;

        double adjDistance = Math.hypot(
                adjGoalX - robotX,
                adjGoalY - robotY
        );

        currentDistance = adjDistance;

        double zoneScale = 1.0;

        targetRPM = tuningMode
                ? tuningRPM
                : shooterEquation.getTargetRPM(adjDistance)
                * zoneScale;

        double baseHoodPos = tuningMode
                ? tuningHoodPos
                : shooterEquation.getHoodPos(adjDistance);

        rpmDrop = Math.max(
                0.0,
                targetRPM - currentVelRPM
        );

        double recoveryDeadband = autonMode
                ? AUTON_TRANSFER_RECOVERY_DEADBAND_RPM
                : TRANSFER_RECOVERY_DEADBAND_RPM;

        double recoveryMultiplier = autonMode
                ? AUTON_TRANSFER_RECOVERY_MULTIPLIER
                : TRANSFER_RECOVERY_MULTIPLIER;

        double activeRecoveryDrop = transferActive
                ? Math.max(0.0, rpmDrop - recoveryDeadband)
                : 0.0;

        correctedTargetRPM =
                targetRPM + activeRecoveryDrop * recoveryMultiplier;

        hoodDrop = 0.0;
        hoodPos = baseHoodPos;

        // PIDF
    }

    public void setTuningMode(boolean enabled) {
        tuningMode = enabled;
    }

    public void setAutonMode(boolean enabled) {
        autonMode = enabled;
    }

    public boolean isReady() {
        return targetRPM > 0.0 && atTarget();
    }

    public boolean isReadyForAuton() {
        return targetRPM > 0.0
                && currentVelRPM >
                targetRPM - AUTON_READY_LOW_RPM_THRESHOLD
                && currentVelRPM <
                targetRPM + RPM_THRESHOLD / 3.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCorrectedTargetRPM() {
        return correctedTargetRPM;
    }

    public double getCurrentVelRPM() {
        return currentVelRPM;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    private boolean atTarget() {
        return currentVelRPM > targetRPM - RPM_THRESHOLD
                && currentVelRPM < targetRPM + RPM_THRESHOLD / 3.0;
    }

    private static double tpsToRPM(double tps) {
        return tps / 28.0 * 60.0;
    }
}
