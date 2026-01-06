package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


/* srihaasa is very intelligent
*/

public class ShootGPPCommand extends SequentialCommandGroup {
    public ShootGPPCommand(Kicker kicker, Spindex spindex, OuttakeColorSensor outtakeCD)
    {


        addCommands(
                new WaitCommand(100),
                new CheckOuttakeAlignmentCommand(spindex, outtakeCD),
                new GoToGreenCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new GoToPurpleCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new GoToPurpleCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new TimedSmallStepCommand(spindex)
        );
        addRequirements(kicker, spindex, outtakeCD);
    }

}