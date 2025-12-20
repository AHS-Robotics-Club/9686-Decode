package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;

import java.util.concurrent.TimeUnit;

public class TimedKickCommand extends CommandBase {

    // The subsystem the command runs on
    private final Kicker kicker;

    private NanoTimer kickTimer;

    public TimedKickCommand(Kicker subsystem) {
        kicker = subsystem;
        addRequirements(kicker);

        kickTimer = new NanoTimer();
    }

    @Override
    public void initialize() {
        kicker.kick();
        kickTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {

        if (kickTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.KICKER_KICK_TIME){

            kicker.down();
            return true;


        }


        return false;
    }

}