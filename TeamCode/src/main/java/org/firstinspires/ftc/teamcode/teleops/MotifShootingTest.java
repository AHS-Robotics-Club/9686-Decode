package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.EmptyChamberCommand;
import org.firstinspires.ftc.teamcode.commands.MotifCommand;
import org.firstinspires.ftc.teamcode.commands.ShootGPPCommand;
import org.firstinspires.ftc.teamcode.commands.ShootPGPCommand;
import org.firstinspires.ftc.teamcode.commands.ShootPPGCommand;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@TeleOp(name = "Sorting Test")
public class MotifShootingTest extends CommandOpMode {

    private Kicker kicker;
    private Spindex spindex;
    private Flywheel flywheel;
    private Intake intake;
    private OuttakeColorSensor outtakeCD;
    private GamepadEx driverPad;

    public char[] ppg = {'P', 'P', 'G'};
    public char[] pgp = {'P', 'G', 'P'};
    public char[] gpp = {'G', 'P', 'P'};

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
                .whenPressed(new MotifCommand(spindex, kicker, outtakeCD, ppg));

        driverPad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new MotifCommand(spindex, kicker, outtakeCD, pgp));

        driverPad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new MotifCommand(spindex, kicker, outtakeCD, gpp));


        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ShootPPGCommand(kicker, spindex, outtakeCD));

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ShootPGPCommand(kicker, spindex, outtakeCD));

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ShootGPPCommand(kicker, spindex, outtakeCD));



    }



    public void run() {

        super.run();


        flywheel.setTargetVeloTicks(-1300);

    }
}
