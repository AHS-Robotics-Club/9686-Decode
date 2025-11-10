package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemFC;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubsystemFC drive;

    private DoubleSupplier leftY, leftX, rightX;

    private double mult;

    public DriveCommand(DriveSubsystemFC drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, double mult){
        this.drive = drive;
        this.leftY = leftY;
        this.leftX = leftX;
        this.rightX = rightX;

        this.mult = mult;

        addRequirements(drive);
    }

    @Override
    public void execute(){drive.drive(leftX.getAsDouble() * mult, leftY.getAsDouble() * mult, rightX.getAsDouble() * mult);
    }


}




