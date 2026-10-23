package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Intake_SS;

public class Intake_Cmd {

    private final Intake_SS intake;
    private Mode currentMode = Mode.IDLE;

    private enum Mode {
        IDLE,
        INTAKING,
        OUTTAKING,
        OPEN_GATE,
        TRANSFERRING
    }

    public Intake_Cmd(Intake_SS intake) {
        this.intake = intake;
    }

    public void update(double robotX, double robotY,
                       boolean shooterReady, boolean aimedAtTarget) {
        update(robotX, robotY, 0.0, shooterReady, aimedAtTarget);
    }

    public void update(double robotX, double robotY, double robotHeading,
                       boolean shooterReady, boolean aimedAtTarget) {
        switch (currentMode) {
            case INTAKING:
                intake.intakeCMD();
                break;

            case OUTTAKING:
                intake.outtakeCMD();
                break;

            case OPEN_GATE:
                intake.openGateCMD();
                break;

            case TRANSFERRING:
                intake.transferCMD(robotX, robotY, robotHeading, shooterReady, aimedAtTarget);
                break;

            case IDLE:
            default:
                intake.stop();   // ← THIS is what was missing
                break;
        }
    }

    public void intake() {
        if (currentMode != Mode.INTAKING) {
            intake.resetIntake();
        }
        setMode(Mode.INTAKING);
    }

    public void outtake() {
        setMode(Mode.OUTTAKING);
    }

    public void openGate() {
        setMode(Mode.OPEN_GATE);
    }

    public void transfer() {
        setMode(Mode.TRANSFERRING);
    }

    public void idle() {
        setMode(Mode.IDLE);
    }

    public boolean isIntaking()     { return currentMode == Mode.INTAKING;     }
    public boolean isOuttaking()    { return currentMode == Mode.OUTTAKING;    }
    public boolean isOpeningGate()  { return currentMode == Mode.OPEN_GATE;    }
    public boolean isTransferring() { return currentMode == Mode.TRANSFERRING; }
    public Intake_SS.BallState getBallState() { return intake.getBallState();   }
    public Intake_SS.TransferState getTransferState() { return intake.getTransferState(); }
    public String getTransferBlockReason() { return intake.getTransferBlockReason(); }
    public boolean isFull() { return intake.isFull(); }
    public String getState() {
        if (currentMode == Mode.TRANSFERRING) {
            return currentMode + " / " + intake.getTransferState()
                    + " / " + intake.getTransferBlockReason();
        }
        if (currentMode == Mode.INTAKING) {
            return currentMode + " / " + intake.getBallState();
        }
        return currentMode.toString();
    }

    private void setMode(Mode mode) {
        currentMode = mode;
    }
}