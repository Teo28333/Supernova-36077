package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import kotlin.jvm.internal.PropertyReference0Impl;

/*
tres important: FRONTSPIN
 */
public class ShooterSS {
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

    private boolean inShootingZone = false;
    private boolean tuningMode = false;
    private boolean autonMode = false;

    public ShooterSS(HardwareMap hwm, Telemetry telemetry,
                 String shooterMotor1, String shooterMotor2,
                 String hoodServo) {
        this.telemetry = telemetry;

        motor1 = hwm.get(DcMotorEx.class, shooterMotor1);
        motor2 = hwm.get(DcMotorEx.class, shooterMotor2);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = hwm.get(Servo.class, hoodServo);
        led = hwm.get(Servo.class, "led2");

        /* a reactiver quand j'ai trouve comment mettre pidfControllers

        pidfsController = new PIDFSController(hwm);
         */
    }

    public void read() {
        currentVelRPM = Math.abs(motor1.getVelocity());
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

    }

    public void telemetry() {
        telemetry.addData("Shooter target (RPM)", targetRPM);
        telemetry.addData("Shooter corrected target (RPM)", correctedTargetRPM);
        telemetry.addData("Shooter current (RPM)", currentVelRPM);
        telemetry.addData("Shooter error (RPM)", targetRPM - currentVelRPM);
        telemetry.addData("Shooter RPM drop", rpmDrop);
        telemetry.addData("Shooter power", motorPow);
        telemetry.addData("Hood recovery drop", hoodDrop);
        telemetry.addData("Hood position", hoodPos);
        telemetry.addData("Distance (in)", currentDistance);
        telemetry.addData("Tuning mode", tuningMode);
    }

    public void shooterSpinCMD(double robotX, double robotY, double robotHeading,
                               Vector robotVel, double goalX, double goalY) {
        shooterSpinCMD(robotX, robotY, robotHeading, robotVel, goalX, goalY, false);
    }

    public void shooterSpinCMD(double robotX, double robotY, double robotHeading,
                               Vector robotVel, double goalX, double goalY,
                               boolean transferActive) {
        if (robotVel == null) {
            robotVel = new Vector(0, 0);
        }


    }
}

//pas fini








