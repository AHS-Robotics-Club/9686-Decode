package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.GoToGreenCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.TimedKickCommand;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@TeleOp(name = "Go to purple test")
public class GoToPurpleTest extends CommandOpMode {

    private Spindex spindex;
    private OuttakeColorSensor outtakeCD;

    private GamepadEx driverPad;

    @Override
    public void initialize() {

        spindex = new Spindex(hardwareMap);
        outtakeCD = new OuttakeColorSensor(hardwareMap);
        register(spindex);
        register(outtakeCD);

        driverPad = new GamepadEx(gamepad1);



        driverPad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new GoToPurpleCommand(spindex, outtakeCD));

        driverPad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new GoToGreenCommand(spindex, outtakeCD));


//        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new TimedKickCommand(kicker));

//        // ✅ Bind button ONCE
//        if (gamepad1.right_trigger != 0) {
//             schedule(new TimedKickCommand(kicker));
//        }

//        new Trigger(() -> gamepad1.right_trigger > 0.5)
//                .whenActive(new TimedKickCommand(kicker));

    }



    public void run() {

        super.run();

//        if (gamepad1.right_trigger > 0.2) {
//            schedule(new TimedKickCommand(kicker));
        }

    }

