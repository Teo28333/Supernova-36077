package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.IntakeConstants.*;
import static org.firstinspires.ftc.teamcode.constants.LedColor.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.LedColor;

public class Intake_SS {
    private final DcMotorEx intake1;
    private final DcMotorEx intake2;
    private final Servo gate;
    private final Servo led;
    private final Telemetry telemetry;

    private final ElapsedTime motor1Timer = new ElapsedTime();
    private final ElapsedTime motor2Timer = new ElapsedTime();
    private final ElapsedTime gateFailsafe = new ElapsedTime();

    private double motor1Current = 0.0;
    private double motor2Current = 0.0;

    private double firstMotorPow = 0.0;
    private double secondMotorPow = 0.0;
    private double closeGatePos = 1.0;

    private double gatePos = closeGatePos;
    private double ledColor = INTAKE_IDLE;

    private boolean motor1Stopped = false;
    private boolean motor2Stopped = false;
    private boolean needToTurn = false;
    private boolean isIntaking = false;
    private boolean intakeStarted = false;
    private boolean isTransferring = false;
    private boolean gateClosingForIntake = false;
    private boolean gateOpeningForTransfer = false;
    private boolean ignoreShootingZoneCheck = false;
    private boolean ignoreAimCheck = false;
    private boolean transferFeedLatched = false;
    private BallState ballState = BallState.EMPTY;
    private TransferState transferState = TransferState.IDLE;
    private String transferBlockReason = "idle";

    public enum BallState {
        EMPTY,
        ONE,
        FULL
    }

    private void updateBallState() {
        if (motor1Stopped) {
            ballState = BallState.FULL;

        } else if (motor2Stopped) {
            ballState = BallState.ONE;

        } else {
            ballState = BallState.EMPTY;
        }
    }

    public enum TransferState {
        IDLE,
        WAITING_GATE,
        WAITING_READY,
        FEEDING
    }

    public Intake_SS(HardwareMap hwm, Telemetry telemetry,
                     String intakeMotor1, String intakeMotor2,
                     String gateServo, String led1) {
        this.telemetry = telemetry;

        intake1 = hwm.get(DcMotorEx.class, intakeMotor1);
        intake2 = hwm.get(DcMotorEx.class, intakeMotor2);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gate = hwm.get(Servo.class, gateServo);
        gatePos = closeGatePos;

        led = hwm.get(Servo.class, led1);
        led.setPosition(ledColor);
    }

    public void read() {
        motor2Current = intake2.getCurrent(CurrentUnit.MILLIAMPS);
        if (motor2Stopped || isTransferring) {
            motor1Current = intake1.getCurrent(CurrentUnit.MILLIAMPS);
        } else {
            motor1Current = 0.0;
        }
    }

    public void write() {
        intake1.setPower(firstMotorPow);
        intake2.setPower(secondMotorPow);
        gate.setPosition(gatePos);
        led.setPosition(ledColor);
    }

    public void telemetry() {
        telemetry.addData("Motor 1 current (mA)", motor1Current);
        telemetry.addData("Motor 2 current (mA)", motor2Current);
        telemetry.addData("Motor 1 stopped", motor1Stopped);
        telemetry.addData("Motor 2 stopped", motor2Stopped);
        telemetry.addData("Intake ball state", ballState);
        telemetry.addData("Transfer state", transferState);
        telemetry.addData("Gate position", gatePos);
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

    public void intakeCMD() {
        if (!intakeStarted) {
            motor1Timer.reset();
            motor2Timer.reset();

            motor1Stopped = false;
            motor2Stopped = false;
            ballState = BallState.EMPTY;

            intakeStarted = true;
        }

        if (!isIntaking) {
            gateClosingForIntake = false;
        }
        isIntaking = true;
        isTransferring = false;
        if (!gateClosingForIntake || gatePos != closeGatePos) {
            gateFailsafe.reset();
            gateClosingForIntake = true;
        }
        gateOpeningForTransfer = false;
        transferState = TransferState.IDLE;
        gatePos = closeGatePos;

        if (gateFailsafe.milliseconds() < gateSettleTime) {
            firstMotorPow = 0.0;
            secondMotorPow = 0.0;
            ledColor = 0.3;
            return;
        }

        if (motor2Current > firstCurrentThreshold) {
            if (motor2Timer.milliseconds() > motor2StopThreshold) motor2Stopped = true;
        } else {
            motor2Timer.reset();
        }

        // Motor 1 ball detection
        if (motor1Current > secondCurrentThreshold && motor2Stopped) {
            if (motor1Timer.milliseconds() > motor1StopThreshold) motor1Stopped = true;
        } else {
            motor1Timer.reset();
        }

        updateBallState();

        secondMotorPow = motor2Stopped ? 0.0 : secondIntakeSpeed;
        firstMotorPow  = motor1Stopped ? 0.0 : intakeSpeed;

        if      (motor1Stopped) ledColor = INTAKE_BALL_MOTOR1;
        else if (motor2Stopped) ledColor = INTAKE_BALL_MOTOR2;
        else                    ledColor = INTAKE_IDLE;
        }

    public void outtakeCMD() {
        isIntaking = false;
        intakeStarted = false;
        isTransferring = false;
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        transferState = TransferState.IDLE;
        gatePos = openGatePos;

        resetIntake();
        ballState = BallState.EMPTY;
        firstMotorPow  = outtakeSpeed;
        secondMotorPow = outtakeSpeed;
    }

    public void openGateCMD() {
        isIntaking = false;
        intakeStarted = false;
        isTransferring = false;
        gateClosingForIntake = false;
        gateOpeningForTransfer = true;
        transferState = TransferState.IDLE;
        gatePos = openGatePos;

        firstMotorPow = 0.0;
        secondMotorPow = 0.0;
        ledColor = INTAKE_IDLE;
    }

    public void transferCMD(double x, double y, double headingRadians,
                            boolean ready, boolean aimedAtTarget) {
        isIntaking = false;
        intakeStarted = false;
        boolean canAim = aimedAtTarget || ignoreAimCheck;
        needToTurn = ready && !canAim;

        if (!isTransferring) {
            resetIntake();
            transferState = TransferState.WAITING_GATE;
            transferBlockReason = "opening gate";
            isTransferring = true;
            gateClosingForIntake = false;
            gateOpeningForTransfer = false;
        }

        if (!gateOpeningForTransfer || gatePos != openGatePos) {
            gateFailsafe.reset();
            gateOpeningForTransfer = true;
        }
        gatePos = openGatePos;

        boolean gateReady = gateFailsafe.milliseconds() >= gateSettleTime;
        if (ready && canAim && gateReady) {
            transferFeedLatched = true;
        }

        if (!transferFeedLatched) {
            firstMotorPow  = 0.0;
            secondMotorPow = 0.0;


            if (!gateReady) {
                transferState = TransferState.WAITING_GATE;
                transferBlockReason = "opening gate";

            } else if (!ready) {
                transferState = TransferState.WAITING_READY;
                transferBlockReason = "shooter not ready";

            } else if (!canAim) {
                transferState = TransferState.WAITING_READY;
                transferBlockReason = "aim not ready";
            }
            return;
        }


        double transferSpeed = IntakeConstants.transferSpeed;

        firstMotorPow = transferSpeed;
        secondMotorPow = transferSpeed;

        transferState = TransferState.FEEDING;
        transferBlockReason = "feeding";
    }

    public void stop() {
        firstMotorPow = 0.0;
        secondMotorPow = 0.0;
        gatePos        = closeGatePos;
        ledColor       = INTAKE_IDLE;
        isIntaking = false;
        intakeStarted = false;
        isTransferring = false;
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        needToTurn     = false;
        transferState  = TransferState.IDLE;
        transferFeedLatched = false;
        transferBlockReason = "idle";
    }

    public void resetIntake() {
        motor1Stopped = false;
        motor2Stopped = false;
        ballState = BallState.EMPTY;
        gateClosingForIntake = false;
        gateOpeningForTransfer = false;
        transferFeedLatched = false;
        transferBlockReason = "idle";
        motor1Timer.reset();
        motor2Timer.reset();
    }

    public BallState getBallState() {
        return ballState;
    }

    public TransferState getTransferState() {
        return transferState;
    }

    public String getTransferBlockReason() {
        return transferBlockReason;
    }

    public boolean isFull() {
        return ballState == BallState.FULL;
    }

}

