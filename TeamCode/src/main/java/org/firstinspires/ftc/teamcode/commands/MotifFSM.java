package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.util.NanoTimer;
import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class MotifFSM {

    /* ===================== Subsystems ===================== */

    private final Spindex spindex;
    private final Kicker kicker;
    private final OuttakeColorSensor outtakeColor;

    /* ======================= Timers ======================= */

    private final NanoTimer rotationTimer = new NanoTimer();
    private final NanoTimer checkTimer = new NanoTimer();
    private final NanoTimer kickTimer = new NanoTimer();
    private final NanoTimer downTimer = new NanoTimer();

    /* ===================== FSM State ====================== */

    private enum State {
        IDLE,
        ALIGN,
        ROTATION_WAIT,
        CHECK_PENDING,
        CHECK,
        KICK,
        LOWER,
        ADVANCE,
        DONE
    }

    private State state = State.IDLE;

    /* ===================== Motif Data ===================== */

    private char[] motif = new char[0];
    private int shotIndex = 0;

    /* ==================== Constructor ===================== */

    public MotifFSM(Spindex spindex, Kicker kicker, OuttakeColorSensor outtakeCD) {
        this.spindex = spindex;
        this.kicker = kicker;
        this.outtakeColor = outtakeCD;
    }

    /* ===================== Lifecycle ====================== */

    /** Start running the motif from the beginning */
    public void start(char[] motif) {
        this.motif = motif;
        shotIndex = 0;

        rotationTimer.resetTimer();
        checkTimer.resetTimer();
        kickTimer.resetTimer();
        downTimer.resetTimer();

        state = State.ALIGN;
    }

    /** Call every OpMode loop */
    public void update() {

        if (state == State.IDLE || state == State.DONE) return;

        switch (state) {

            case ALIGN:
                // Step forward to align ball in shooter slot
                if (outtakeColor.getDistance()
                        > RobotConstraints.OUTTAKE_BALL_POSITION_THRESH) {

                    spindex.stepForward();
                    rotationTimer.resetTimer();
                    state = State.ROTATION_WAIT;

                } else {
                    // Already aligned
                    checkTimer.resetTimer();
                    state = State.CHECK_PENDING;
                }
                break;

            case CHECK_PENDING:
                state = State.CHECK;
                break;

            case ROTATION_WAIT:
                // Wait for spindex rotation to stabilize the ball position
                if (rotationTimer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.SPINDEX_120_DEG_COLOR_STAB_TIME) {

                    checkTimer.resetTimer();
                    state = State.CHECK;
                }
                break;

            case CHECK:
                // Wait for color sensor to stabilize readings
                if (checkTimer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.OUTTAKE_CD_READING_TIME) {

                    char desiredColor = motif[shotIndex];

                    boolean isGreen =
                            outtakeColor.getHue()
                                    < RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH;

                    boolean isPurple =
                            outtakeColor.getHue()
                                    >= RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH;

                    boolean match =
                            (desiredColor == 'G' && isGreen)
                                    || (desiredColor == 'P' && isPurple);

                    if (match) {
                        // Ball matches motif, shoot it
                        kicker.kick();
                        kickTimer.resetTimer();
                        state = State.KICK;

                    } else {
                        // Ball does not match, rotate to next
                        spindex.bigStepForward();
                        rotationTimer.resetTimer();
                        state = State.ROTATION_WAIT;
                    }
                }
                break;

            case KICK:
                if (kickTimer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.KICKER_KICK_TIME) {

                    kicker.down();
                    downTimer.resetTimer();
                    state = State.LOWER;
                }
                break;

            case LOWER:
                if (downTimer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.KICKER_DOWN_TIME) {

                    state = State.ADVANCE;
                }
                break;

            case ADVANCE:
                shotIndex++;

                if (shotIndex >= motif.length) {
                    state = State.DONE;
                } else {
                    spindex.bigStepForward();
                    rotationTimer.resetTimer();
                    state = State.ROTATION_WAIT;
                }
                break;

            case DONE:
                break;
        }
    }

    /* ===================== Status API ===================== */

    public boolean isActive() {
        return state != State.IDLE && state != State.DONE;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public void stop() {
        kicker.down();
        state = State.IDLE;
    }
}
