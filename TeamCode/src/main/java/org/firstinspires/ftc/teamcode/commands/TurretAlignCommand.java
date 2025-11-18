package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretAlignCommand extends CommandBase {
    private final Turret turret;
    private final Limelight limelight;

    public TurretAlignCommand(Turret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret, limelight);
    }

    @Override
    public void execute() {
        // Only try to aim if the camera actually sees something!
        if (limelight.hasTarget()) {
            turret.autoAim(limelight.getTx());
        } else {
            // Safety: Stop if we lose the tag
            turret.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command finishes (user releases button)
        turret.stop();
    }
}