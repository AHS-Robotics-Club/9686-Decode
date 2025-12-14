package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;


public class EmptyChamberCommand extends CommandBase {

    private final Kicker kicker;
    private final Spindex spindex;
    private final IntakeColorSensor intakeCD;

    private int totalShots;
    private int shotsFired;
    private NanoTimer transferTimer;
    private boolean counting;

    public EmptyChamberCommand(Kicker kicker, Spindex spindex, IntakeColorSensor intakeCD, int totalShots) {
        this.kicker = kicker;
        this.spindex = spindex;
        this.intakeCD = intakeCD;

        this.totalShots = Math.min(totalShots, 3); // max 3 balls
        this.shotsFired = 0;
        this.transferTimer = new NanoTimer();
        this.counting = false;

        addRequirements(kicker, spindex);
    }

    @Override
    public void initialize() {
        // Start first shot immediately if a ball is present
        if (intakeCD.getDistance() > RobotConstraints.INTAKE_BALL_CHAMBERED_DISTANCE) {
            startNextShot();
        }
    }

    @Override
    public void execute() {
        // Only proceed if a shot is in progress
        if (counting && transferTimer.getElapsedTime() > RobotConstraints.SPINDEX_120_DEG_ROT_TIME) {
            startNextShot();
        }
    }

    private void startNextShot() {
        if (shotsFired >= totalShots) {
            counting = false; // done
            return;
        }

        // Fire kicker
        kicker.kick();
        kicker.down();

        // Step spindex
        spindex.bigStepForward();

        // Prepare for next shot
        shotsFired++;
        counting = true;
        transferTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return shotsFired >= totalShots && !counting;
    }
}
