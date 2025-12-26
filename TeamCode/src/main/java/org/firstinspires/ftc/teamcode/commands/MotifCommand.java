package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class MotifCommand extends CommandBase {

    private final Spindex spindex;
    private final Kicker kicker;
    private final OuttakeColorSensor outtakeCD;

    private final NanoTimer rotationTimer = new NanoTimer();
    private final NanoTimer checkTimer = new NanoTimer();
    private final NanoTimer kickTimer = new NanoTimer();
    private final NanoTimer downTimer = new NanoTimer();

    private int shotIndex = 0; // how many balls shot
    private final char[] motif; // e.g., {'P','G','P'}


    private enum State {
        ALIGN,          // step forward to align ball
        ROTATION_WAIT,  // wait for spindex rotation to complete (120°)
        CHECK,          // check color for 75ms

        CHECK_PENDING,
        KICK,           // kicker kicking
        LOWER,          // kicker lowering
        ADVANCE,        // move to next shot
        DONE
    }

    private State state = State.ALIGN;

    public MotifCommand(Spindex spindex, Kicker kicker, OuttakeColorSensor outtakeCD, char[] motif) {
        this.spindex = spindex;
        this.kicker = kicker;
        this.outtakeCD = outtakeCD;
        this.motif = motif;
        addRequirements(spindex, kicker, outtakeCD);
    }

    @Override
    public void initialize() {
        shotIndex = 0;
        state = State.ALIGN;
        rotationTimer.resetTimer();

        kickTimer.resetTimer();
        downTimer.resetTimer();
    }

    @Override
    public void execute() {


        switch (state) {

            case ALIGN:
                // Step forward to align ball in shooter slot
                if (outtakeCD.getDistance() > RobotConstraints.OUTTAKE_BALL_POSITION_THRESH) {
                    spindex.stepForward();
                    rotationTimer.resetTimer();
                    state = State.ROTATION_WAIT;
                    return;
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
                if (rotationTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.SPINDEX_120_DEG_COLOR_STAB_TIME) {
                    checkTimer.resetTimer();
                    state = State.CHECK;
                }
                break;

            case CHECK:
                // Wait for color sensor to stabilize readings
                if (checkTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.OUTTAKE_CD_READING_TIME) {
                    char desiredColor = motif[shotIndex];
                    boolean isGreen = outtakeCD.getHue() < RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH;
                    boolean isPurple = outtakeCD.getHue() >= RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH;

                    boolean match = (desiredColor == 'G' && isGreen) || (desiredColor == 'P' && isPurple);

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
                if (kickTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.KICKER_KICK_TIME) {
                    kicker.down();
                    downTimer.resetTimer();
                    state = State.LOWER;
                }
                break;

            case LOWER:
                if (downTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.KICKER_DOWN_TIME) {
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
                // Nothing to do
                break;
        }
    }



    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }
}
