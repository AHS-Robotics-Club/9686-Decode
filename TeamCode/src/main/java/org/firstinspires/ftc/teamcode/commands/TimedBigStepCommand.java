package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class TimedBigStepCommand extends CommandBase {

    // The subsystem the command runs on
    private final Spindex spindex;

    private NanoTimer cycleTimer;

    public TimedBigStepCommand(Spindex subsystem) {
        spindex = subsystem;
        addRequirements(spindex);

        cycleTimer = new NanoTimer();
    }

    @Override
    public void initialize() {
        spindex.bigStepForward();
        cycleTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {

        if (cycleTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.SPINDEX_120_DEG_ROT_TIME){


            return true;


        }


        return false;
    }

}