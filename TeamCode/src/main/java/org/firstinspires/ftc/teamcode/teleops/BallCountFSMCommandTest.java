package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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

    private Intake intake;

    private Flywheel flywheel;

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

    private NanoTimer cycleTimer = new NanoTimer();

    /* ===================== Shoot FSM ===================== */
    private enum ShootState {
        IDLE,
        ALIGNING,
        SMALL_ALIGNING,
        KICKING,
        WAIT_CLEAR
    }
    private ShootState shootState = ShootState.IDLE;
    private NanoTimer kickTimer = new NanoTimer();

    private boolean shootRequested = false;
    private boolean isShooting = false;


    /* ===================== Tunables ===================== */
    private static final double INTAKE_BALL_DISTANCE = 50.0;
    private static final double GREEN_THRESHOLD = 0.0145;
    private static final double SPINDEX_BIG_TIME = 300;

    private static final double OUTTAKE_BALL_DISTANCE = 40.0;
    private static final double SPINDEX_STEP_TIME = 0.22;
    private static final double KICK_TIME = 120;

    private static final double KICKER_DOWN_TIME = 150;

    @Override
    public void initialize() {
        spindex = new Spindex(hardwareMap);
        kicker = new Kicker(hardwareMap);
        intakeCD = new IntakeColorSensor(hardwareMap);
        outtakeCD = new OuttakeColorSensor(hardwareMap);
        intake = new Intake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);

        register(spindex);
        register(kicker);
        register(intake);
        register(intakeCD);
        register(outtakeCD);
        register(flywheel);

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

        flywheel.setTargetVeloTicks(-800);


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
                    cycleTimer.resetTimer();

                    intakeState = IntakeState.INDEXING;
                }
                break;

            case INDEXING:
                if (cycleTimer.getElapsedTime() >= RobotConstraints.SPINDEX_120_DEG_ROT_TIME) {
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
        boolean hasBalls = totalBalls() > 0;

        switch (shootState) {
            case IDLE:
                if (isShooting) break;

                if (shootRequested && hasBalls) {
                    isShooting = true;
                    shootRequested = false;

                    boolean ballInChamber = outtakeCD.getDistance() < RobotConstraints.OUTTAKE_BALL_POSITION_THRESH;

                    if (!ballInChamber) {
                        // Need to move spindex to bring next ball into chamber
                        spindex.stepForward();
                        cycleTimer.resetTimer(); // timer for spindex rotation
                        shootState = ShootState.SMALL_ALIGNING;

                        telemetry.addLine("IDLE -> SMALL_ALIGNING");
                    } else {
                        // Ball already in chamber, kick immediately
                        kicker.kick();
                        kickTimer.resetTimer(); // timer for kicker extension
                        shootState = ShootState.KICKING;

                        telemetry.addLine("IDLE -> KICKING");
                    }
                }
                break;

            case SMALL_ALIGNING:
                if (cycleTimer.getElapsedTime() >= RobotConstraints.SPINDEX_60_DEG_ROT_TIME) {
                    kicker.kick();
                    kickTimer.resetTimer();
                    shootState = ShootState.KICKING;

                    telemetry.addLine("SMALL_ALIGNING -> KICKING");
                }
                break;

            case ALIGNING:
                if (cycleTimer.getElapsedTime() >= RobotConstraints.SPINDEX_120_DEG_ROT_TIME) {
                    kicker.kick();
                    kickTimer.resetTimer();
                    shootState = ShootState.KICKING;

                    telemetry.addLine("ALIGNING -> KICKING");
                }
                break;

            case KICKING:
                if (kickTimer.getElapsedTime() >= RobotConstraints.KICKER_KICK_TIME) {
                    kicker.down();
                    kickTimer.resetTimer(); // wait for kicker down
                    shootState = ShootState.WAIT_CLEAR;

                    telemetry.addLine("KICKING -> WAIT_CLEAR");
                }
                break;

            case WAIT_CLEAR:
                boolean ballCleared = outtakeCD.getDistance() >= RobotConstraints.OUTTAKE_BALL_POSITION_THRESH;

                // Only decrement once per shot
                if (ballCleared && hasBalls) {
                    decrementBallCount();
                    telemetry.addLine("Ball decremented");
                }

                if (ballCleared) {
                    if (totalBalls() > 0) {
                        // More balls remaining, advance spindex and align
                        spindex.bigStepForward();
                        cycleTimer.resetTimer();
                        shootState = ShootState.ALIGNING;

                        telemetry.addLine("WAIT_CLEAR -> ALIGNING");
                    } else {
                        // Done shooting
                        isShooting = false;
                        shootState = ShootState.IDLE;

                        telemetry.addLine("WAIT_CLEAR -> IDLE");
                    }
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
