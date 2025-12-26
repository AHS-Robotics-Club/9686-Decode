package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


/* srihaasa is very intelligent
*/

public class ShootPGPCommand extends SequentialCommandGroup {
    public ShootPGPCommand(Kicker kicker, Spindex spindex, OuttakeColorSensor outtakeCD)
    {


        addCommands(
                new CheckOuttakeAlignmentCommand(spindex, outtakeCD),
                new GoToPurpleCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new GoToGreenCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new GoToPurpleCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new TimedSmallStepCommand(spindex)
        );
        addRequirements(kicker, spindex, outtakeCD);
    }

}