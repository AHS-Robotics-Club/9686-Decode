package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;

@TeleOp(name = "Ball Manager FSM Full Test", group = "TEST")
public class BallCountFSMCommandTest extends CommandOpMode {

    /* ===================== Subsystems ===================== */
    private Spindex spindex;
    private Kicker kicker;
    private IntakeColorSensor intakeCD;
    private OuttakeColorSensor outtakeCD;

    /* ===================== Intake FSM ===================== */
    private enum IntakeState {
        IDLE,
        INDEXING,
        CONFIRM,
        WAIT_CLEAR
    }
    private IntakeState intakeState = IntakeState.IDLE;

    private boolean pendingGreen = false;
    private boolean pendingPurple = false;

    private int numGreenBalls = 0;
    private int numPurpleBalls = 0;

    private NanoTimer intakeTimer = new NanoTimer();

    /* ===================== Shoot FSM ===================== */
    private enum ShootState {
        IDLE,
        ALIGNING,
        KICKING,
        WAIT_CLEAR
    }
    private ShootState shootState = ShootState.IDLE;
    private NanoTimer shootTimer = new NanoTimer();

    private boolean shootRequested = false;
    private boolean isShooting = false;
    private boolean kickerDownComplete = false;

    /* ===================== Tunables ===================== */
    private static final double INTAKE_BALL_DISTANCE = 50.0;
    private static final double GREEN_THRESHOLD = 0.0145;
    private static final double SPINDEX_BIG_TIME = 0.28;

    private static final double OUTTAKE_BALL_DISTANCE = 40.0;
    private static final double SPINDEX_STEP_TIME = 0.22;
    private static final double KICK_TIME = 0.12;

    private static final double KICKER_DOWN_TIME = 0.15;

    @Override
    public void initialize() {
        spindex = new Spindex(hardwareMap);
        kicker = new Kicker(hardwareMap);
        intakeCD = new IntakeColorSensor(hardwareMap);
        outtakeCD = new OuttakeColorSensor(hardwareMap);

        register(spindex);
        register(kicker);

        telemetry.addLine("Ball Manager FSM Full Test READY");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // CommandScheduler

        // Manual trigger to empty chamber
        if (gamepad2.a && !isShooting && totalBalls() > 0) {
            shootRequested = true;
        }

        updateShootFSM();
        updateIntakeFSM();
        sendTelemetry();
    }

    /* ===================== Intake FSM ===================== */
    private void updateIntakeFSM() {
        if (isShooting) return; // BLOCK intake while shooting

        double distance = intakeCD.getDistance();
        double green = intakeCD.getGreen();

        boolean ballPresent = distance > 5 && distance < INTAKE_BALL_DISTANCE;
        boolean isGreen = ballPresent && green > GREEN_THRESHOLD;

        switch (intakeState) {
            case IDLE:
                if (ballPresent && totalBalls() < 3) {
                    pendingGreen = isGreen;
                    pendingPurple = !isGreen;

                    spindex.bigStepForward();
                    intakeTimer.resetTimer();

                    intakeState = IntakeState.INDEXING;
                }
                break;

            case INDEXING:
                if (intakeTimer.getElapsedTime() >= SPINDEX_BIG_TIME) {
                    intakeState = IntakeState.CONFIRM;
                }
                break;

            case CONFIRM:
                if (pendingGreen) numGreenBalls++;
                if (pendingPurple) numPurpleBalls++;

                pendingGreen = false;
                pendingPurple = false;

                intakeState = IntakeState.WAIT_CLEAR;
                break;

            case WAIT_CLEAR:
                if (!ballPresent) {
                    intakeState = IntakeState.IDLE;
                }
                break;
        }
    }

    /* ===================== Shoot FSM ===================== */
    private void updateShootFSM() {
        boolean ballInChamber = outtakeCD.getDistance() < OUTTAKE_BALL_DISTANCE;
        boolean hasBalls = totalBalls() > 0;

        switch (shootState) {
            case IDLE:
                if (shootRequested && hasBalls) {
                    isShooting = true;
                    shootRequested = false;
                    kickerDownComplete = false;

                    if (!ballInChamber) {
                        spindex.stepForward();
                        shootTimer.resetTimer(); // start ALIGNING wait
                        shootState = ShootState.ALIGNING;
                    } else {
                        kicker.kick();
                        shootTimer.resetTimer(); // start KICKING wait
                        shootState = ShootState.KICKING;
                    }
                }
                break;

            case ALIGNING:
                // Wait for spindex to physically rotate
                if (shootTimer.getElapsedTime() >= SPINDEX_STEP_TIME) {
                    kicker.kick();
                    shootTimer.resetTimer(); // start KICKING wait
                    shootState = ShootState.KICKING;
                }
                break;

            case KICKING:
                // Wait for kicker to fully extend
                if (shootTimer.getElapsedTime() >= KICK_TIME) {
                    kicker.down();
                    shootTimer.resetTimer(); // start WAIT_CLEAR delay for kicker down
                    shootState = ShootState.WAIT_CLEAR;
                }
                break;

            case WAIT_CLEAR:
                // Wait for kicker to be fully down
                if (!kickerDownComplete && shootTimer.getElapsedTime() >= KICKER_DOWN_TIME) {
                    decrementBallCount(); // only once
                    kickerDownComplete = true;
                }

                // Wait for chamber to be clear before returning to IDLE
                if (kickerDownComplete && !ballInChamber) {
                    if (hasBalls) {
                        shootRequested = true; // auto-continue shooting
                    } else {
                        isShooting = false; // unlock intake
                    }
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    /* ===================== Helpers ===================== */
    private int totalBalls() {
        return numGreenBalls + numPurpleBalls;
    }

    private void decrementBallCount() {
        if (numGreenBalls > 0) {
            numGreenBalls--;
        } else if (numPurpleBalls > 0) {
            numPurpleBalls--;
        }
    }

    /* ===================== Telemetry ===================== */
    private void sendTelemetry() {
        telemetry.addData("INTAKE FSM", intakeState);
        telemetry.addData("SHOOT FSM", shootState);

        telemetry.addData("Intake Dist", intakeCD.getDistance());
        telemetry.addData("Outtake Dist", outtakeCD.getDistance());
        telemetry.addData("Green Val", intakeCD.getGreen());

        telemetry.addData("Green Balls", numGreenBalls);
        telemetry.addData("Purple Balls", numPurpleBalls);
        telemetry.addData("Total Balls", totalBalls());

        telemetry.addData("Is Shooting", isShooting);
        telemetry.update();
    }
}
