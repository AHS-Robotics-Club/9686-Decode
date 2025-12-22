package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.EmptyChamberCommand;
import org.firstinspires.ftc.teamcode.commands.TimedKickCommand;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@TeleOp(name = "EmptyChamberTest")
public class EmptyChamberTest extends CommandOpMode {

    private Kicker kicker;
    private Spindex spindex;
    private Flywheel flywheel;
    private Intake intake;
    private OuttakeColorSensor outtakeCD;
    private GamepadEx driverPad;

    @Override
    public void initialize() {

        kicker = new Kicker(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        outtakeCD = new OuttakeColorSensor(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        register(kicker);
        register(flywheel);
        register(spindex);
        register(outtakeCD);
        register(intake);


        driverPad = new GamepadEx(gamepad1);



        driverPad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new EmptyChamberCommand(kicker, spindex, outtakeCD));



    }



    public void run() {

        super.run();


        flywheel.setTargetVeloTicks(-1300);

    }
}
