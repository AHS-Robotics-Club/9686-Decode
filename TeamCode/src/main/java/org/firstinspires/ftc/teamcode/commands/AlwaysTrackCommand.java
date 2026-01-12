package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class AlwaysTrackCommand extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;

    public AlwaysTrackCommand(Turret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;
        // This is crucial: it tells the scheduler this command uses the Turret
        addRequirements(turret);
        // Note: We usually DON'T add Limelight as a requirement for read-only commands
        // so other commands can also read from Limelight if needed.
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            // If we see a tag, use your P-Controller to aim
            turret.autoAim(limelight.getTx());
        } else {
            turret.stop();
        }
    }

    // This command never finishes on its own because we want it running forever
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

}