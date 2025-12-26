package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class GoToPurpleCommand extends CommandBase {

    private enum State { ROTATE, WAIT, CHECK, FINISHED }

    private State state = State.ROTATE;
    private Spindex spindex;
    private  OuttakeColorSensor outtakeCD;
    private final NanoTimer cycleTimer;
    private final NanoTimer checkTimer;


    private boolean loopedOnce;

    public GoToPurpleCommand(Spindex spindex, OuttakeColorSensor outtakeCD) {
        this.spindex = spindex;
        this.outtakeCD = outtakeCD;
        cycleTimer = new NanoTimer();
        checkTimer = new NanoTimer();

        addRequirements(spindex, outtakeCD);
    }

    @Override
    public void initialize() {
        state = State.CHECK;
        checkTimer.resetTimer();
    }

    @Override
    public void execute() {
        switch(state) {
            case ROTATE:
                // Already rotated, start waiting
                cycleTimer.resetTimer();
                state = State.WAIT;
                break;

            case WAIT:
                if(cycleTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.SPINDEX_120_DEG_COLOR_STAB_TIME) {
                    checkTimer.resetTimer();
                    state = State.CHECK;
                }
                break;

            case CHECK:
                double colorValue = outtakeCD.getHue(); // or whatever metric you use
                if(colorValue > RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH && outtakeCD.getDistance() < RobotConstraints.OUTTAKE_BALL_POSITION_THRESH) {
                    state = State.FINISHED;
                } else if (checkTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.OUTTAKE_CD_READING_TIME){
                    spindex.bigStepForward();
                    state = State.WAIT;
                    cycleTimer.resetTimer();
                }
                break;

            case FINISHED:
                // nothing to do
                break;
        }
    }

    @Override
    public boolean isFinished() {


        if (!loopedOnce) {
            loopedOnce = true;
            return false;


        }


        return state == State.FINISHED && checkTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.OUTTAKE_CD_READING_TIME && loopedOnce;

    }
}
