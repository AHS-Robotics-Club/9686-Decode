package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Robot;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class SimpleShootFSM {

    private final Spindex spindex;
    private final Kicker kicker;

    private final NanoTimer timer = new NanoTimer();
    private IntakeColorSensor intakeCD;

    private enum State {

        DELAY,
        ALIGN,
        ROTATE_WAIT,
        KICK,
        KICK_HOLD,
        LOWER,
        ADVANCE,
        DONE
    }

    private State state = State.DELAY;
    private int shotsFired = 0;
    private final int totalShots;

    public SimpleShootFSM(Spindex spindex, Kicker kicker, int totalShots, IntakeColorSensor intakeCD) {
        this.spindex = spindex;
        this.kicker = kicker;
        this.totalShots = totalShots;
        this.intakeCD = intakeCD;
    }

    public void start() {
        shotsFired = 0;
        state = State.DELAY;
        timer.resetTimer();
    }

    public void update() {

        if (state == State.DONE) return;

        switch (state) {

            case DELAY:

                if (timer.getElapsedTime(TimeUnit.MILLISECONDS) > 3500) {

                    state = State.ALIGN;
                }



            case ALIGN:
                if (intakeCD.getDistance()
                        < RobotConstraints.INTAKE_BALL_CHAMBERED_DISTANCE) {

                    spindex.stepForward();
                    timer.resetTimer();
                    state = State.ROTATE_WAIT;

                } else {
                    // Already aligned
                    timer.resetTimer();
                    state = State.KICK;
                }

            case ROTATE_WAIT:
                if (timer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.SPINDEX_120_DEG_COLOR_STAB_TIME) {

                    kicker.kick();
                    timer.resetTimer();
                    state = State.KICK;
                }
                break;

            case KICK:


                if (timer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.KICKER_KICK_TIME) {
                    kicker.down();
                    timer.resetTimer();
                    state = State.LOWER;
                }
                break;

            case KICK_HOLD:
                if (timer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.KICKER_DOWN_TIME) {

                    kicker.down();
                    timer.resetTimer();
                    state = State.LOWER;
                }
                break;

            case LOWER:
                if (timer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.KICKER_DOWN_TIME) {

                    state = State.ADVANCE;
                }
                break;

            case ADVANCE:
                shotsFired++;

                if (shotsFired >= totalShots) {
                    state = State.DONE;
                } else {
                    spindex.bigStepForward();
                    timer.resetTimer();
                    state = State.ROTATE_WAIT;   // force rotation before next kick
                }
                break;
        }
    }

    public boolean isDone() {
        return state == State.DONE;
    }
}
