package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import java.util.concurrent.TimeUnit;

public class CheckOuttakeAlignmentCommand extends CommandBase {

    private OuttakeColorSensor outtakeCD;
    private Spindex spindex;

    private NanoTimer cycleTimer;

    private boolean aligned;




    public CheckOuttakeAlignmentCommand(Spindex spindex, OuttakeColorSensor outtakeCD) {
        this.spindex = spindex;
        this.outtakeCD = outtakeCD;
        addRequirements(spindex, outtakeCD);

        cycleTimer = new NanoTimer();



    }


    @Override
    public void initialize() {

        if (outtakeCD.getDistance() > RobotConstraints.OUTTAKE_BALL_POSITION_THRESH) {
            spindex.stepForward();
            cycleTimer.resetTimer();

            aligned = false;
        } else {
            aligned = true;
        }

    }

    @Override
    public boolean isFinished() {

        if (!aligned && outtakeCD.getDistance() < RobotConstraints.OUTTAKE_BALL_POSITION_THRESH &&
                cycleTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.SPINDEX_60_DEG_ROT_TIME) {


            return true;


        } else if (aligned) {
            return true;
        }


        return false;
    }




}
