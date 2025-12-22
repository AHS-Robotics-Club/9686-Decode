package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


/* srihaasa is very intelligent
*/

public class EmptyChamberCommand extends SequentialCommandGroup {
    public EmptyChamberCommand(Kicker kicker, Spindex spindex, OuttakeColorSensor outtakeCD)
    {


        addCommands(
                new CheckOuttakeAlignmentCommand(spindex, outtakeCD),
                new TimedKickCommand(kicker),
                new TimedBigStepCommand(spindex),
                new TimedKickCommand(kicker),
                new TimedBigStepCommand(spindex),
                new TimedKickCommand(kicker),
                new TimedSmallStepCommand(spindex)
        );
        addRequirements(kicker, spindex, outtakeCD);
    }

}