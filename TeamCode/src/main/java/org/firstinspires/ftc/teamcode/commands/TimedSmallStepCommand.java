package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class TimedSmallStepCommand extends CommandBase {

    // The subsystem the command runs on
    private final Spindex spindex;

    private NanoTimer cycleTimer;

    public TimedSmallStepCommand(Spindex subsystem) {
        spindex = subsystem;
        addRequirements(spindex);

        cycleTimer = new NanoTimer();
    }

    @Override
    public void initialize() {
        spindex.stepForward();
        cycleTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {

        if (cycleTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.SPINDEX_60_DEG_ROT_TIME){


            return true;


        }


        return false;
    }

}