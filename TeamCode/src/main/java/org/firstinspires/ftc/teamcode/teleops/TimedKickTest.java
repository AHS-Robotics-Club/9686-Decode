package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TimedKickCommand;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;

@TeleOp(name = "TimedKickTest")
public class TimedKickTest extends CommandOpMode {

    private Kicker kicker;
    private GamepadEx driverPad;

    @Override
    public void initialize() {

        kicker = new Kicker(hardwareMap);
        register(kicker);

        driverPad = new GamepadEx(gamepad1);



        driverPad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new TimedKickCommand(kicker));


        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new TimedKickCommand(kicker));

//        // ✅ Bind button ONCE
//        if (gamepad1.right_trigger != 0) {
//             schedule(new TimedKickCommand(kicker));
//        }

        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new TimedKickCommand(kicker));

    }



    public void run() {

        super.run();

        if (gamepad1.right_trigger > 0.2) {
            schedule(new TimedKickCommand(kicker));
        }

    }
}
